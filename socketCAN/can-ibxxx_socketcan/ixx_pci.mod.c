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

MODULE_ALIAS("pci:v00001BEEd00000002sv00001BEEsd00000002bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000013sv00001BEEsd00000013bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000010sv00001BEEsd00000010bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000015sv00001BEEsd00000015bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000004sv00001BEEsd00000004bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000005sv00001BEEsd00000005bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd0000000Esv00001BEEsd0000000Ebc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000017sv00001BEEsd00000017bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000012sv00001BEEsd00000012bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000019sv00001BEEsd00000019bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd0000001Asv00001BEEsd0000001Abc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000003sv00001BEEsd00000003bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000014sv00001BEEsd00000014bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000011sv00001BEEsd00000011bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000016sv00001BEEsd00000016bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000006sv00001BEEsd00000006bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd0000000Fsv00001BEEsd0000000Fbc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd00000018sv00001BEEsd00000018bc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd0000001Bsv00001BEEsd0000001Bbc*sc*i*");
MODULE_ALIAS("pci:v00001BEEd0000001Csv00001BEEsd0000001Cbc*sc*i*");

MODULE_INFO(srcversion, "94DF548A4006E2D72DFFD09");
