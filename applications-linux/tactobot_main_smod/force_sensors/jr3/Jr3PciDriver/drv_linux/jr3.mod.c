#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x9a31bb74, "module_layout" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x4c4fef19, "kernel_stack" },
	{ 0x385fa373, "pci_release_region" },
	{ 0x9cc5adfc, "__register_chrdev" },
	{ 0x27e1a049, "printk" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0x42c8de35, "ioremap_nocache" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xf55bb238, "pci_unregister_driver" },
	{ 0x4f68e5c9, "do_gettimeofday" },
	{ 0xaba33759, "__pci_register_driver" },
	{ 0x5a4896a8, "__put_user_2" },
	{ 0x8f9c199c, "__get_user_2" },
	{ 0xace5f3c3, "pci_enable_device" },
	{ 0x79e8df86, "pci_request_region" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v00001762d00001111sv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "48BB2EDE1C31B5D990C3616");
