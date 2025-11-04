#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
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



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xbbba5367, "pci_irq_vector" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x3a1b27aa, "_dev_err" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x1b9275a7, "__dynamic_dev_dbg" },
	{ 0xfdb9a5b1, "free_netdev" },
	{ 0x23fbc566, "__netif_napi_del" },
	{ 0x609f1c7e, "synchronize_net" },
	{ 0xdac1fff4, "pci_disable_device" },
	{ 0xf4d27367, "pci_free_irq_vectors" },
	{ 0x3d012277, "unregister_netdev" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xfd3de8a2, "pci_unregister_driver" },
	{ 0x27bbf221, "disable_irq_nosync" },
	{ 0x32b5678f, "napi_schedule_prep" },
	{ 0xeff94b95, "__napi_schedule" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x9d08530e, "netif_carrier_on" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0xbfc5b7bd, "dma_unmap_page_attrs" },
	{ 0xa103eed9, "skb_put" },
	{ 0x72993552, "eth_type_trans" },
	{ 0xe91cc893, "napi_gro_receive" },
	{ 0x37a0cba, "kfree" },
	{ 0x122c3a7e, "_printk" },
	{ 0x2828b7f8, "napi_complete_done" },
	{ 0xfcec0987, "enable_irq" },
	{ 0x965da639, "__netdev_alloc_skb" },
	{ 0xc31db0ce, "is_vmalloc_addr" },
	{ 0x7cd8d75e, "page_offset_base" },
	{ 0x97651e6c, "vmemmap_base" },
	{ 0xa1b34704, "dma_map_page_attrs" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0x37a99944, "kmalloc_caches" },
	{ 0x22e14f04, "kmalloc_trace" },
	{ 0x692894f7, "dev_driver_string" },
	{ 0x56470118, "__warn_printk" },
	{ 0x360e778b, "dev_kfree_skb_any_reason" },
	{ 0x4c9d28b0, "phys_base" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x57f19d91, "netif_tx_wake_queue" },
	{ 0x2cf56265, "__dynamic_pr_debug" },
	{ 0x2ae736be, "netif_tx_lock" },
	{ 0x7ec2b7e5, "netif_tx_unlock" },
	{ 0xe523ad75, "synchronize_irq" },
	{ 0xe2964344, "__wake_up" },
	{ 0xe2c17b5d, "__SCT__might_resched" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x6dca08ed, "napi_enable" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x27e4991c, "napi_disable" },
	{ 0x3ce4ca6f, "disable_irq" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x8b091dd, "__pci_register_driver" },
	{ 0x95a35de, "alloc_etherdev_mqs" },
	{ 0x41ed3709, "get_random_bytes" },
	{ 0xaab8d67a, "dev_addr_mod" },
	{ 0x3d53ea68, "pci_enable_device" },
	{ 0xc8e6536b, "devm_ioremap" },
	{ 0x28917712, "pci_set_master" },
	{ 0x6ae9bef6, "netif_napi_add_weight" },
	{ 0xbd3d6b81, "register_netdev" },
	{ 0x9c42dfe0, "netif_carrier_off" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x4d44d354, "pci_alloc_irq_vectors" },
	{ 0xc6227e48, "module_layout" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "A4714EA3201F9521C133EF6");
