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

#include <stdlib.h>

#include <libxl.h>
#include <libxl_utils.h>
#include <libxlutil.h>

#include "xl.h"
#include "xl_utils.h"
#include "xl_parse.h"

int main_vi2cattach(int argc, char **argv)
{
    int opt;
    int rc;
    uint32_t domid;
    libxl_device_vi2c vi2c;

    SWITCH_FOREACH_OPT(opt, "", NULL, "vi2c-attach", 1) {
        /* No options */
    }

    libxl_device_vi2c_init(&vi2c);
    domid = find_domain(argv[optind++]);

    for (argv += optind, argc -= optind; argc > 0; ++argv, --argc) {
        rc = parse_vi2c_config(&vi2c, *argv);
        if (rc) goto out;
    }

    if (vi2c.num_slaves == 0) {
        fprintf(stderr, "At least one slave should be specified.\n");
        rc = ERROR_FAIL; goto out;
    }

    if (dryrun_only) {
        char *json = libxl_device_vi2c_to_json(ctx, &vi2c);
        printf("vi2c: %s\n", json);
        free(json);
        rc = 0;
        goto out;
    }

    if (libxl_device_vi2c_add(ctx, domid, &vi2c, 0)) {
        fprintf(stderr, "libxl_device_vi2c_add failed.\n");
        rc = ERROR_FAIL; goto out;
    }

    rc = 0;

out:
    libxl_device_vi2c_dispose(&vi2c);
    return rc;
}

int main_vi2clist(int argc, char **argv)
{
   int opt;
   int i, j, n;
   libxl_device_vi2c *vi2cs;
   libxl_vi2cinfo vi2cinfo;

   SWITCH_FOREACH_OPT(opt, "", NULL, "vi2c-list", 1) {
       /* No options */
   }

   for (argv += optind, argc -= optind; argc > 0; --argc, ++argv) {
       uint32_t domid;

       if (libxl_domain_qualifier_to_domid(ctx, *argv, &domid) < 0) {
           fprintf(stderr, "%s is an invalid domain identifier\n", *argv);
           continue;
       }

       vi2cs = libxl_device_vi2c_list(ctx, domid, &n);

       if (!vi2cs) continue;

       for (i = 0; i < n; i++) {
           libxl_vi2cinfo_init(&vi2cinfo);
           if (libxl_device_vi2c_getinfo(ctx, domid, &vi2cs[i],
                                           &vi2cinfo) == 0) {
               printf("DevId: %d, BE: %d, handle: %d, state: %d, "
                      "be-adapter: %s, BE-path: %s, FE-path: %s\n",
                       vi2cinfo.devid, vi2cinfo.backend_id,
                       vi2cinfo.frontend_id,
                       vi2cinfo.state, vi2cinfo.be_adapter,
                       vi2cinfo.backend, vi2cinfo.frontend);
               for (j = 0; j < vi2cinfo.num_slaves; j++) {
                   printf("\tslave: 0x%x\n", vi2cinfo.slaves[j]);
               }
           }
           libxl_vi2cinfo_dispose(&vi2cinfo);
       }
       libxl_device_vi2c_list_free(vi2cs, n);
   }
   return 0;
}

int main_vi2cdetach(int argc, char **argv)
{
    uint32_t domid, devid;
    int opt, rc;
    libxl_device_vi2c vi2c;

    SWITCH_FOREACH_OPT(opt, "", NULL, "vi2c-detach", 2) {
        /* No options */
    }

    domid = find_domain(argv[optind++]);
    devid = atoi(argv[optind++]);

    libxl_device_vi2c_init(&vi2c);

    if (libxl_devid_to_device_vi2c(ctx, domid, devid, &vi2c)) {
        fprintf(stderr, "Error: Device %d not connected.\n", devid);
        rc = ERROR_FAIL;
        goto out;
    }

    rc = libxl_device_vi2c_remove(ctx, domid, &vi2c, 0);
    if (rc) {
        fprintf(stderr, "libxl_device_vi2c_remove failed.\n");
        rc = ERROR_FAIL;
        goto out;
    }

    rc = 0;

out:
    libxl_device_vi2c_dispose(&vi2c);
    return rc;
}

/*
 * Local variables:
 * mode: C
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
