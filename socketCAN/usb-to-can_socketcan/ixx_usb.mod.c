#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(.gnu.linkonce.this_module) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

MODULE_INFO(depends, "can-dev");

MODULE_ALIAS("usb:v08D8p0008d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0009d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p000Ad*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p000Bd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p000Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p000Dd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0017d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0014d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0016d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p001Bd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p001Cd*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "D0B699355735EFD3AA81E2E");
