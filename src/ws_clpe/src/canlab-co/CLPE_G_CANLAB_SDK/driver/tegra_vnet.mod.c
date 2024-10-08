#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

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
	{ 0x30ff7695, "module_layout" },
	{ 0xc0b2bc48, "pci_unregister_driver" },
	{ 0x433f0b06, "__pci_register_driver" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xe523ad75, "synchronize_irq" },
	{ 0x733968a0, "netif_schedule_queue" },
	{ 0x33a21a09, "pv_ops" },
	{ 0x7a2af7b4, "cpu_number" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0xaf708deb, "netif_tx_wake_queue" },
	{ 0x4d756883, "napi_enable" },
	{ 0x6d881d1, "__netdev_alloc_skb" },
	{ 0x4f00afd3, "kmem_cache_alloc_trace" },
	{ 0xac1c4313, "kmalloc_caches" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x4c9d28b0, "phys_base" },
	{ 0x56470118, "__warn_printk" },
	{ 0xad7b2e29, "dev_driver_string" },
	{ 0x837b7b09, "__dynamic_pr_debug" },
	{ 0x337c1f75, "__dev_kfree_skb_any" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x461f3cc3, "dma_map_page_attrs" },
	{ 0x97651e6c, "vmemmap_base" },
	{ 0x7cd8d75e, "page_offset_base" },
	{ 0xc31db0ce, "is_vmalloc_addr" },
	{ 0xfcec0987, "enable_irq" },
	{ 0x1ec5bac3, "napi_complete_done" },
	{ 0x37a0cba, "kfree" },
	{ 0x26636414, "napi_gro_receive" },
	{ 0x9a0f02d3, "eth_type_trans" },
	{ 0x3df865a5, "skb_put" },
	{ 0x4b4ce6ce, "dma_unmap_page_attrs" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x9b37b411, "__napi_schedule" },
	{ 0x3e9d3fa8, "napi_schedule_prep" },
	{ 0x27bbf221, "disable_irq_nosync" },
	{ 0x3d519628, "_dev_err" },
	{ 0x43f931fc, "__dynamic_dev_dbg" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x5be158a5, "pci_alloc_irq_vectors_affinity" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xcd526a0, "register_netdev" },
	{ 0x9606736d, "netif_napi_add" },
	{ 0xb24af53b, "pci_set_master" },
	{ 0x15bd5079, "devm_ioremap" },
	{ 0x28cd97e, "pci_enable_pcie_error_reporting" },
	{ 0x58a78df9, "pci_enable_device" },
	{ 0x41ed3709, "get_random_bytes" },
	{ 0x5eccf71d, "alloc_etherdev_mqs" },
	{ 0x93ba01c5, "free_netdev" },
	{ 0x3d258838, "pci_disable_device" },
	{ 0x609f1c7e, "synchronize_net" },
	{ 0x6e2f755, "__netif_napi_del" },
	{ 0xacce876c, "unregister_netdev" },
	{ 0x8b5bc5b3, "pci_free_irq_vectors" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x3ce4ca6f, "disable_irq" },
	{ 0x45a2728c, "pci_irq_vector" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x800473f, "__cond_resched" },
	{ 0xe85a634c, "napi_disable" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0xe4fe6f93, "netif_carrier_on" },
	{ 0xb1d4c565, "netif_carrier_off" },
	{ 0x92997ed8, "_printk" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xbdfb6dbb, "__fentry__" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "7E87303EA39F686729D8088");
