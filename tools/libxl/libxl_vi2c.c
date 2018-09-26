/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 only. with the special
 * exception on linking described in file LICENSE.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#include "libxl_internal.h"

static int libxl__device_vi2c_setdefault(libxl__gc *gc, uint32_t domid,
                                           libxl_device_vi2c *vi2c,
                                           bool hotplug)
{
    return libxl__resolve_domid(gc, vi2c->backend_domname,
                                &vi2c->backend_domid);
}

static int libxl__vi2c_from_xenstore(libxl__gc *gc, const char *libxl_path,
                                       libxl_devid devid,
                                       libxl_device_vi2c *vi2c)
{
    const char *be_path;
    int rc;

    vi2c->devid = devid;
    rc = libxl__xs_read_mandatory(gc, XBT_NULL,
                                  GCSPRINTF("%s/backend", libxl_path),
                                  &be_path);
    if (rc) return rc;

    return libxl__backendpath_parse_domid(gc, be_path, &vi2c->backend_domid);
}

static int libxl_device_vi2c_compare(libxl_device_vi2c *d1,
                                       libxl_device_vi2c *d2)
{
    return COMPARE_DEVID(d1, d2);
}

static void libxl__device_vi2c_add(libxl__egc *egc, uint32_t domid,
                                     libxl_device_vi2c *vi2c,
                                     libxl__ao_device *aodev)
{
    libxl__device_add_async(egc, domid, &libxl__vi2c_devtype, vi2c, aodev);
}

static int libxl_vi2c_adapter_to_id(libxl__gc *gc, char *adapter)
{
    DIR *dir;
    struct dirent *de;
    int adapter_id = -1;

    dir = opendir(SYSFS_I2C_ADAPTER);
    if (!dir) {
        LOGE(ERROR, "opendir failed: '%s'", SYSFS_I2C_ADAPTER);
	return ERROR_FAIL;
    }

    for (;;) {
        char *id, *p, *s;
        char *filename;
        void *buf;

        errno = 0;
        de = readdir(dir);
        if (!de && errno) {
            LOGE(ERROR, "failed to readdir %s", SYSFS_I2C_ADAPTER);
            break;
        }
        if (!de)
            break;

        if (!strcmp(de->d_name, ".") ||
            !strcmp(de->d_name, ".."))
            continue;

        filename = GCSPRINTF(SYSFS_I2C_ADAPTER "/%s/name", de->d_name);
        if (!libxl__read_sysfs_file_contents(gc, filename, &buf, NULL)) {
            if (!strncmp(adapter, buf, strlen(adapter))) {
                s = libxl__strdup(gc, de->d_name);
                p = strtok_r(s, "-", &id);
		if (p == NULL)
                    continue;
		adapter_id = atoi(id);
		break;
            }
        } else {
                return ERROR_FAIL;
        }
    }

    closedir(dir);

    return adapter_id;
}

static int libxl__set_xenstore_vi2c(libxl__gc *gc, uint32_t domid,
                                    libxl_device_vi2c *vi2c,
                                    flexarray_t *back, flexarray_t *front,
                                    flexarray_t *ro_front)
{
    int adapter_id;
    int i;

    adapter_id = libxl_vi2c_adapter_to_id(gc, vi2c->be_adapter);
    if (adapter_id < 0)
        return adapter_id;

    flexarray_append_pair(ro_front, "be-adapter",
                          GCSPRINTF("%s", vi2c->be_adapter));
    flexarray_append_pair(back, "adapter",
                          GCSPRINTF("%d", adapter_id));
    flexarray_append_pair(back, "num-slaves",
                          GCSPRINTF("%d", vi2c->num_slaves));
    for (i = 0; i < vi2c->num_slaves; i++)
        flexarray_append_pair(back, GCSPRINTF("%d/addr", i),
                              GCSPRINTF("%x", vi2c->slaves[i]));

    return 0;
}

static int libxl__device_vi2c_getslaves(libxl_ctx *ctx, const char *path,
                                        libxl_vi2cinfo *info)
{
    GC_INIT(ctx);
    char *slave = NULL;
    char *slave_path;
    int i, rc;

    info->num_slaves = 0;

    slave_path = GCSPRINTF("%s/%d", path, info->num_slaves);

    while ((slave = xs_read(ctx->xsh, XBT_NULL, slave_path, NULL)) !=
           NULL) {
        free(slave);
        slave_path = GCSPRINTF("%s/%d", path, ++info->num_slaves);
    }

    info->slaves = libxl__calloc(NOGC, info->num_slaves,
                                 sizeof(*info->slaves));

    for (i = 0; i < info->num_slaves; i++) {
        char *value_path;
        char *value;

        value_path = GCSPRINTF("%s/%d/addr", path, i);
        value = xs_read(ctx->xsh, XBT_NULL, value_path, NULL);
        if (value == NULL) { rc = ERROR_FAIL; goto out; }

        info->slaves[i] = strtoul(value, NULL, 16);
        free(value);
    }

    rc = 0;
out:
    return rc;
}

int libxl_device_vi2c_getinfo(libxl_ctx *ctx, uint32_t domid,
                                libxl_device_vi2c *vi2c,
                                libxl_vi2cinfo *info)
{
    GC_INIT(ctx);
    char *libxl_path, *devpath;
    char *val;
    int rc;

    libxl_vi2cinfo_init(info);
    info->devid = vi2c->devid;

    devpath = libxl__domain_device_frontend_path(gc, domid, info->devid,
                                                 LIBXL__DEVICE_KIND_VI2C);
    libxl_path = libxl__domain_device_libxl_path(gc, domid, info->devid,
                                                 LIBXL__DEVICE_KIND_VI2C);

    info->backend = xs_read(ctx->xsh, XBT_NULL,
                            GCSPRINTF("%s/backend", libxl_path),
                            NULL);
    if (!info->backend) { rc = ERROR_FAIL; goto out; }

    rc = libxl__backendpath_parse_domid(gc, info->backend, &info->backend_id);
    if (rc) goto out;

    val = libxl__xs_read(gc, XBT_NULL, GCSPRINTF("%s/state", devpath));
    info->state = val ? strtoul(val, NULL, 10) : -1;

    val = libxl__xs_read(gc, XBT_NULL, GCSPRINTF("%s/event-channel", devpath));
    info->evtch = val ? strtoul(val, NULL, 10) : -1;

    val = libxl__xs_read(gc, XBT_NULL, GCSPRINTF("%s/ring-ref", devpath));
    info->rref = val ? strtoul(val, NULL, 10) : -1;

    info->frontend = xs_read(ctx->xsh, XBT_NULL,
                             GCSPRINTF("%s/frontend", libxl_path),
                             NULL);
    info->frontend_id = domid;

    info->be_adapter = libxl__xs_read(gc, XBT_NULL,
                                      GCSPRINTF("%s/be-adapter", devpath));

    rc = libxl__device_vi2c_getslaves(ctx, devpath, info);
    if (rc) goto out;

    rc = 0;

out:
     GC_FREE;
     return rc;
}

static LIBXL_DEFINE_DEVICE_FROM_TYPE(vi2c)
static LIBXL_DEFINE_UPDATE_DEVID(vi2c)
static LIBXL_DEFINE_DEVICES_ADD(vi2c)

LIBXL_DEFINE_DEVID_TO_DEVICE(vi2c)
LIBXL_DEFINE_DEVICE_ADD(vi2c)
LIBXL_DEFINE_DEVICE_REMOVE(vi2c)
LIBXL_DEFINE_DEVICE_LIST(vi2c)

DEFINE_DEVICE_TYPE_STRUCT(vi2c, VI2C,
    .from_xenstore = (device_from_xenstore_fn_t)libxl__vi2c_from_xenstore,
    .set_xenstore_config = (device_set_xenstore_config_fn_t)
                           libxl__set_xenstore_vi2c
);

/*
 * Local variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
