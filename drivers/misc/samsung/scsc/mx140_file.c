/****************************************************************************
 *
 *   Copyright (c) 2016 Samsung Electronics Co., Ltd. All rights reserved.
 *
 ****************************************************************************/

#include <linux/module.h>
#include <linux/version.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <scsc/scsc_logring.h>
#include <scsc/scsc_mx.h>

#include "scsc_mx_impl.h"

/* Firmware directory definitions */

#if defined(CONFIG_SCSC_CORE_FW_LOCATION) && !defined(CONFIG_SCSC_CORE_FW_LOCATION_AUTO)
#define MX140_FW_BASE_DIR_SYSTEM_ETC_WIFI      CONFIG_SCSC_CORE_FW_LOCATION
#define MX140_FW_BASE_DIR_VENDOR_ETC_WIFI      CONFIG_SCSC_CORE_FW_LOCATION
#else
#define MX140_FW_BASE_DIR_SYSTEM_ETC_WIFI	"/system/etc/wifi"
#define MX140_FW_BASE_DIR_VENDOR_ETC_WIFI	"/vendor/etc/wifi"
#endif

/* Look for this file in <dir>/etc/wifi */
#define MX140_FW_DETECT				"mx"

/* Paths for vendor utilities, used when CONFIG_SCSC_CORE_FW_LOCATION_AUTO=n */
#define MX140_EXE_DIR_VENDOR		"/vendor/bin"    /* Oreo */
#define MX140_EXE_DIR_SYSTEM		"/system/bin"	 /* Before Oreo */

#define MX140_FW_CONF_SUBDIR            "conf"
#define MX140_FW_DEBUG_SUBDIR           "debug"
#define MX140_FW_BIN                    "mx140.bin"
#define MX140_FW_PATH_MAX_LENGTH        (512)

#define MX140_FW_VARIANT_DEFAULT        "mx140"

/* Table of suffixes to append to f/w name */
struct fw_suffix {
	char suffix[4];
	u32 hw_ver;
};

/* Table of known RF h/w revs */
static const struct fw_suffix fw_suffixes[] = {
	{ .suffix = "_11", .hw_ver = 0x11, },
	{ .suffix = "_10", .hw_ver = 0x10, },
	{ .suffix = "_00", .hw_ver = 0x00, },
	{ .suffix = "",    .hw_ver = 0xff, }, /* plain mx140.bin, must be last */
};

/* Once set, we always load this firmware suffix */
static int fw_suffix_found = -1;

/* Variant of firmware binary to load */
static char *firmware_variant = MX140_FW_VARIANT_DEFAULT;
module_param(firmware_variant, charp, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(firmware_variant, "mx140 firmware variant, default mx140");

/* RF hardware version of firmware to load. If "auto" this gets replaced with
 * the suffix of FW that got loaded.
 * If "manual" it loads the version specified by firmware_variant, verbatim.
 */
static char *firmware_hw_ver = "auto";
module_param(firmware_hw_ver, charp, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(firmware_hw_ver, "mx140 hw version detect, manual=disable");

/* FW base dir readable by usermode script */
static char *fw_base_dir = CONFIG_SCSC_CORE_FW_LOCATION;
module_param_named(base_dir, fw_base_dir, charp, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(base_dir, "WLBT FW base directory");

/* Firmware and tool (moredump) exe base directory */
#ifdef CONFIG_SCSC_CORE_FW_LOCATION_AUTO
static char base_dir[MX140_FW_PATH_MAX_LENGTH]; /* auto detect */
static char exe_dir[MX140_FW_PATH_MAX_LENGTH];	/* auto detect */
#else
static char base_dir[] = CONFIG_SCSC_CORE_FW_LOCATION;  /* fixed in defconfig */
static char exe_dir[] = CONFIG_SCSC_CORE_TOOL_LOCATION;	/* fixed in defconfig */
#endif


static bool enable_auto_sense;
module_param(enable_auto_sense, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(enable_auto_sense, "deprecated");

static bool use_new_fw_structure = true;
module_param(use_new_fw_structure, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(use_new_fw_structure, "deprecated");

static char *cfg_platform = "default";
module_param(cfg_platform, charp, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(cfg_platform, "HCF config subdirectory");

/* Reads a configuration file into memory (f/w profile specific) */
static int __mx140_file_request_conf(struct scsc_mx *mx,
		const struct firmware **conf,
		const char *platform_dir,
		const char *config_rel_path,
		const char *filename,
		const bool flat)

{
	char config_path[MX140_FW_PATH_MAX_LENGTH];
	int r;

	if (mx140_basedir_file(mx))
		return -ENOENT;

	if (flat) {
		/* e.g. /etc/wifi/mx140_wlan.hcf */

		scnprintf(config_path, sizeof(config_path),
			"%s/%s%s_%s",
			base_dir,
			firmware_variant,
			fw_suffixes[fw_suffix_found].suffix,
			filename);
	} else {
		/* e.g. /etc/wifi/mx140/conf/$platform_dir/wlan/wlan.hcf */

		scnprintf(config_path, sizeof(config_path),
			"%s/%s%s/%s/%s%s%s/%s",
			base_dir,
			firmware_variant,
			fw_suffixes[fw_suffix_found].suffix,
			MX140_FW_CONF_SUBDIR,
			platform_dir,
			(platform_dir[0] != '\0' ? "/" : ""), /* add "/" if platform_dir not empty */
			config_rel_path,
			filename);
	}
	SCSC_TAG_INFO(MX_FILE, "try %s\n", config_path);

	r = mx140_request_file(mx, config_path, conf);

	/* Confirm what we read */
	if (r == 0)
		SCSC_TAG_INFO(MX_FILE, "loaded %s\n", config_path);

	return r;
}

int mx140_file_request_conf(struct scsc_mx *mx,
			    const struct firmware **conf,
			    const char *config_rel_path,
			    const char *filename)
{
	int r;

	/* First, if the config subdirectory has been overriden by cfg_platform
	 * module parameter, search only in that location.
	 */
	if (strcmp(cfg_platform, "default")) {
		SCSC_TAG_INFO(MX_FILE, "module param cfg_platform = %s\n", cfg_platform);
		return __mx140_file_request_conf(mx, conf, cfg_platform, config_rel_path, filename, false);
	}

	/* Search in generic location. This is an override.
	 * e.g. /etc/wifi/mx140/conf/wlan/wlan.hcf
	 */
	r = __mx140_file_request_conf(mx, conf, "", config_rel_path, filename, false);

#if defined CONFIG_SCSC_WLBT_CONFIG_PLATFORM
	/* Then  search in platform location
	 * e.g. /etc/wifi/mx140/conf/$platform_dir/wlan/wlan.hcf
	 */
	if (r) {
		const char *plat = CONFIG_SCSC_WLBT_CONFIG_PLATFORM;

		/* Don't bother if plat is empty string */
		if (plat[0] != '\0')
			r = __mx140_file_request_conf(mx, conf, plat, config_rel_path, filename, false);
	}
#endif

	/* Finally request "flat" conf, where all hcf files are in FW root dir
	 * e.g. /etc/wifi/<firmware-variant>-wlan.hcf
	 */
	if (r)
		r = __mx140_file_request_conf(mx, conf, "", config_rel_path, filename, true);

	return r;
}

EXPORT_SYMBOL(mx140_file_request_conf);

/* Reads a debug configuration file into memory (f/w profile specific) */
int mx140_file_request_debug_conf(struct scsc_mx *mx, const struct firmware **conf, const char *config_rel_path)
{
	char          config_path[MX140_FW_PATH_MAX_LENGTH];

	if (mx140_basedir_file(mx))
		return -ENOENT;

	/* e.g. /etc/wifi/mx140/debug/log_strings.bin */

	scnprintf(config_path, sizeof(config_path), "%s/%s%s/%s/%s",
		base_dir,
		firmware_variant,
		fw_suffixes[fw_suffix_found].suffix,
		MX140_FW_DEBUG_SUBDIR,
		config_rel_path);

	return mx140_request_file(mx, config_path, conf);
}
EXPORT_SYMBOL(mx140_file_request_debug_conf);

/* Read device configuration file into memory (whole device specific) */
int mx140_file_request_device_conf(struct scsc_mx *mx, const struct firmware **conf, const char *config_rel_path)
{
	char          config_path[MX140_FW_PATH_MAX_LENGTH];

	if (mx140_basedir_file(mx))
		return -ENOENT;

	/* e.g. /etc/wifi/conf/wlan/mac.txt */

	snprintf(config_path, sizeof(config_path), "%s/%s%s/%s",
		base_dir,
		fw_suffixes[fw_suffix_found].suffix,
		MX140_FW_CONF_SUBDIR,
		config_rel_path);

	return mx140_request_file(mx, config_path, conf);
}
EXPORT_SYMBOL(mx140_file_request_device_conf);

/* Release configuration file memory. */
void mx140_file_release_conf(struct scsc_mx *mx, const struct firmware *conf)
{
	(void)mx;

	mx140_release_file(mx, conf);
}
EXPORT_SYMBOL(mx140_file_release_conf);

static int __mx140_file_download_fw(struct scsc_mx *mx, void *dest, size_t dest_size, u32 *fw_image_size, const char *fw_suffix)
{
	const struct firmware *firm;
	int                   r = 0;
	char                  img_path_name[MX140_FW_PATH_MAX_LENGTH];

	/* If fs is not mounted yet, may return -EAGAIN, denoting retry later */
	r = mx140_basedir_file(mx);
	if (r)
		return r;

	SCSC_TAG_INFO(MX_FILE, "firmware_variant=%s (%s)\n", firmware_variant, fw_suffix);

	/* e.g. /etc/wifi/mx140.bin */
	scnprintf(img_path_name, sizeof(img_path_name), "%s/%s%s.bin",
		base_dir,
		firmware_variant,
		fw_suffix);

	SCSC_TAG_DEBUG(MX_FILE, "Load CR4 fw %s in shared address %p\n", img_path_name, dest);
	r = mx140_request_file(mx, img_path_name, &firm);
	if (r) {
		SCSC_TAG_ERR(MX_FILE, "Error Loading FW, error %d\n", r);
		return r;
	}
	SCSC_TAG_DEBUG(MX_FILE, "FW Download, size %zu\n", firm->size);

	if (firm->size > dest_size) {
		SCSC_TAG_ERR(MX_FILE, "firmware image too big for buffer (%zu > %u)", dest_size, *fw_image_size);
		r = -EINVAL;
	} else {
		memcpy(dest, firm->data, firm->size);
		*fw_image_size = firm->size;
	}
	mx140_release_file(mx, firm);
	return r;
}

/* Download firmware binary into a buffer supplied by the caller */
int mx140_file_download_fw(struct scsc_mx *mx, void *dest, size_t dest_size, u32 *fw_image_size)
{
	int r;
	int i;
	int manual;

	/* Override to use the verbatim image only */
	manual = !strcmp(firmware_hw_ver, "manual");
	if (manual) {
		SCSC_TAG_INFO(MX_FILE, "manual hw version\n");
		fw_suffix_found = sizeof(fw_suffixes) / sizeof(fw_suffixes[0]) - 1;
	}

	SCSC_TAG_DEBUG(MX_FILE, "fw_suffix_found %d\n", fw_suffix_found);

	/* If we know which f/w suffix to use, select it immediately */
	if (fw_suffix_found != -1) {
		r = __mx140_file_download_fw(mx, dest, dest_size, fw_image_size, fw_suffixes[fw_suffix_found].suffix);
		goto done;
	}

	/* Otherwise try the list */
	for (i = 0; i < sizeof(fw_suffixes) / sizeof(fw_suffixes[0]); i++) {
		/* Try to find each suffix in turn */
		SCSC_TAG_INFO(MX_FILE, "try %d %s\n", i, fw_suffixes[i].suffix);
		r = __mx140_file_download_fw(mx, dest, dest_size, fw_image_size, fw_suffixes[i].suffix);
		if (r != -ENOENT)
			break;
	}

	/* Save this for next time */
	if (r == 0)
		fw_suffix_found = i;
done:
	/* Update firmware_hw_ver to reflect what got auto selected, for moredump */
	if (fw_suffix_found != -1 && !manual) {
		/* User will only read this, so casting away const is safe */
		firmware_hw_ver = (char *)fw_suffixes[fw_suffix_found].suffix;
	}
	return r;
}

int mx140_request_file(struct scsc_mx *mx, char *path, const struct firmware **firmp)
{
	struct file *f;
	mm_segment_t fs;
	struct kstat stat;
	const int max_read_size = 4096;
	int r, whats_left, to_read, size;
	struct firmware *firm;
	char *buf, *p;

	SCSC_TAG_DEBUG(MX_FILE, "request %s\n", path);

	*firmp = NULL;

	/* Check FS is ready */

	/* Try to determine base dir */
	r = mx140_basedir_file(mx);
	if (r) {
		SCSC_TAG_ERR(MX_FILE, "detect failed for fw base_dir %d\n", r);
		return r;
	}

	/* Current segment. */
	fs = get_fs();
	/* Set to kernel segment. */
	set_fs(get_ds());

	r = vfs_stat(base_dir, &stat);
	if (r != 0) {
		set_fs(fs);
		SCSC_TAG_ERR(MX_FILE, "vfs_stat() failed for %s\n", base_dir);
		return -EAGAIN;
	}

	/* Check f/w bin */
	r = vfs_stat(path, &stat);
	if (r != 0) {
		set_fs(fs);
		SCSC_TAG_ERR(MX_FILE, "vfs_stat() failed for %s\n", path);
		return -ENOENT;
	}
	/* Revert to original segment. */
	set_fs(fs);

	/* Round up for minimum sizes */
	size = (stat.size + 256) & ~255;
	/* Get memory for file contents. */
	buf = vzalloc(size);
	if (!buf) {
		SCSC_TAG_ERR(MX_FILE, "kzalloc(%d) failed for %s\n", size, path);
		return -ENOMEM;
	}
	p = buf;
	/* Get firmware structure. */
	firm = kzalloc(sizeof(*firm), GFP_KERNEL);
	if (!firm) {
		vfree(buf);
		SCSC_TAG_ERR(MX_FILE, "kzalloc(%zu) failed for %s\n", sizeof(*firmp), path);
		return -ENOMEM;
	}
	/* Open the file for reading. */
	f = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(f)) {
		vfree(buf);
		kfree(firm);
		SCSC_TAG_ERR(MX_FILE, "filp_open() failed for %s with %ld\n", path, PTR_ERR(f));
		return -ENOENT;
	}

	whats_left = stat.size;

	fs = get_fs();
	set_fs(get_ds());
	/* Read at most max_read_size in each read. Loop until the whole file has
	 * been copied to the local buffer.
	 */
	while (whats_left) {
		to_read = whats_left < max_read_size ? whats_left : max_read_size;
		r = vfs_read(f, p, to_read, &f->f_pos);
		if (r < 0) {
			SCSC_TAG_ERR(MX_FILE, "error reading %s\n", path);
			break;
		}
		if (r == 0 || r < to_read)
			break;
		whats_left -= r;
		p += r;
	}
	set_fs(fs);
	filp_close(f, NULL);

	if (r >= 0) {
		r = 0;
		/* Pass to caller. Caller will free allocated memory through
		 * mx140_release_file().
		 */
		firm->size = p - buf;
		firm->data = buf;
		*firmp = firm;
	} else {
		vfree(buf);
		kfree(firm);
	}
	return r;

}
EXPORT_SYMBOL(mx140_request_file);

int mx140_release_file(struct scsc_mx *mx, const struct firmware *firmp)
{
	if (!firmp || !firmp->data) {
		SCSC_TAG_ERR(MX_FILE, "firmp=%p\n", firmp);
		return -EINVAL;
	}

	SCSC_TAG_DEBUG(MX_FILE, "release firmp=%p, data=%p\n", firmp, firmp->data);

	vfree(firmp->data);
	kfree(firmp);
	return 0;
}
EXPORT_SYMBOL(mx140_release_file);

/* Work out correct path for vendor binaries */
int mx140_exe_path(struct scsc_mx *mx, char *path, size_t len, const char *bin)
{
	(void)mx;

	/* Set up when we detect FW path, or statically when
	 * auto-detect is off
	 */
	if (exe_dir[0] == '\0')
		return -ENOENT;

	if (path == NULL)
		return -EINVAL;

	snprintf(path, len, "%s/%s", exe_dir, bin);

	SCSC_TAG_DEBUG(MX_FILE, "exe: %s\n", path);
	return 0;
}
EXPORT_SYMBOL(mx140_exe_path);

/* Try to auto detect f/w directory */
int mx140_basedir_file(struct scsc_mx *mx)
{
	struct kstat stat;
	mm_segment_t fs;
	int r = 0;

	/* Already worked out base dir. This is
	 * static if auto-detect is off.
	 */
	if (base_dir[0] != '\0')
		return 0;

	/* Default to pre-O bin dir, until we detect O */
	strlcpy(exe_dir, MX140_EXE_DIR_SYSTEM, sizeof(exe_dir));

	/* Current segment. */
	fs = get_fs();
	/* Set to kernel segment. */
	set_fs(get_ds());

/* HACK: Skip mount point check (postmarketOS) */
#if 0
	/* If /system isn't present, assume platform isn't ready yet */
	r = vfs_stat("/system", &stat);
	if (r != 0) {
		SCSC_TAG_ERR(MX_FILE, "/system not mounted yet\n");
		r = -EAGAIN;
		goto done;
	}

	/* If /vendor isn't present, assume O platform isn't ready yet.
	 * Android M and N still have /vendor, though we don't use it.
	 * Searching for /vendor is not enough as it's a mountpoint and
	 * appears before its contents.
	 */
	r = vfs_stat("/vendor", &stat);
	if (r != 0) {
		SCSC_TAG_ERR(MX_FILE, "/vendor not mounted yet\n");
		r = -EAGAIN;
		goto done;
	}
#endif

	/* Search for SCSC FW under the mountpoints */

	/* Try /vendor partition  (Oreo) first.
	 * If it's present, it'll contain our FW
	 */
	r = vfs_stat(MX140_FW_BASE_DIR_VENDOR_ETC_WIFI"/"MX140_FW_DETECT, &stat);
	if (r != 0) {
		SCSC_TAG_ERR(MX_FILE, "Base dir: %s/%s doesn't exist\n",
			MX140_FW_BASE_DIR_VENDOR_ETC_WIFI, MX140_FW_DETECT);
		base_dir[0] = '\0';
		r = -ENOENT;
	} else {
		strlcpy(base_dir, MX140_FW_BASE_DIR_VENDOR_ETC_WIFI, sizeof(base_dir));
		fw_base_dir = MX140_FW_BASE_DIR_VENDOR_ETC_WIFI;
		strlcpy(exe_dir, MX140_EXE_DIR_VENDOR, sizeof(exe_dir));
		goto done;
	}

	/* Try /system partition (pre-Oreo) */
	r = vfs_stat(MX140_FW_BASE_DIR_SYSTEM_ETC_WIFI"/"MX140_FW_DETECT, &stat);
	if (r != 0) {
		SCSC_TAG_ERR(MX_FILE, "Base dir: %s/%s doesn't exist\n",
			MX140_FW_BASE_DIR_SYSTEM_ETC_WIFI, MX140_FW_DETECT);
		base_dir[0] = '\0';

		/* FW not present in O or N locations. We might still be waiting
		 * for the fs to mount inside the /vendor mountpoint, -EAGAIN to
		 * retry. Caller is responsible for deciding when to stop trying.
		 */
		r = -EAGAIN;
	} else {
		strlcpy(base_dir, MX140_FW_BASE_DIR_SYSTEM_ETC_WIFI, sizeof(base_dir));
		fw_base_dir = MX140_FW_BASE_DIR_SYSTEM_ETC_WIFI;
	}

done:
	/* Restore segment */
	set_fs(fs);
	SCSC_TAG_INFO(MX_FILE, "WLBT fw base dir is %s\n", base_dir[0] ? base_dir : "not found");

	return r;
}

/* Select file for h/w version from filesystem */
int mx140_file_select_fw(struct scsc_mx *mx, u32 hw_ver)
{
	int i;

	SCSC_TAG_INFO(MX_FILE, "select f/w for 0x%04x\n", hw_ver);

	hw_ver = (hw_ver & 0xff00) >> 8; /* LSB is the RF HW ID (e.g. S610) */

	for (i = 0; i < sizeof(fw_suffixes) / sizeof(fw_suffixes[0]); i++) {
		if (fw_suffixes[i].hw_ver == hw_ver) {
			fw_suffix_found = i;
			SCSC_TAG_DEBUG(MX_FILE, "f/w for 0x%04x: index %d, suffix '%s'\n",
				hw_ver, i, fw_suffixes[i].suffix);
			return 0;
		}
	}

	SCSC_TAG_ERR(MX_FILE, "No known f/w for 0x%04x, default to catchall\n", hw_ver);

	/* Enable when a unified FW image is installed */
#ifdef MX140_UNIFIED_HW_FW
	/* The last f/w is the non-suffixed "<fw>.bin", assume it's compatible */
	fw_suffix_found = i - 1;
#else
	fw_suffix_found = -1; /* None found */
#endif
	return -EINVAL;
}

/* Query whether this HW is supported by the current FW file set */
bool mx140_file_supported_hw(struct scsc_mx *mx, u32 hw_ver)
{
	hw_ver = (hw_ver & 0xff00) >> 8; /* LSB is the RF HW ID (e.g. S610) */
	/* Assume installed 0xff is always compatible, and f/w will panic if it isn't */
	if (fw_suffixes[fw_suffix_found].hw_ver == 0xff)
		return true;

	/* Does the select f/w match the hw_ver from chip? */
	return (fw_suffixes[fw_suffix_found].hw_ver == hw_ver);
}
