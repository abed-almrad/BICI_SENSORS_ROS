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
	{ 0x133d294f, "module_layout" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x85bd1608, "__request_region" },
	{ 0xa773c546, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0xf90a1e85, "__x86_indirect_thunk_r8" },
	{ 0xc0b3699e, "pci_free_irq_vectors" },
	{ 0x98b2b1fe, "pci_write_config_word" },
	{ 0x50da5547, "single_open" },
	{ 0x77358855, "iomem_resource" },
	{ 0x2a8db161, "dma_set_mask" },
	{ 0xc1cc106b, "single_release" },
	{ 0x310d0b8c, "usb_reset_endpoint" },
	{ 0x2b47ac5e, "pci_disable_device" },
	{ 0x59db2485, "i2c_transfer" },
	{ 0x20000329, "simple_strtoul" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0x9fae7ac4, "seq_printf" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x733605e5, "usb_kill_urb" },
	{ 0xa0db056e, "remove_proc_entry" },
	{ 0xfa83b84e, "device_destroy" },
	{ 0x772807c3, "__register_chrdev" },
	{ 0xabd9b6a4, "driver_for_each_device" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0xcd1472e8, "pci_release_regions" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0xb395e05f, "dma_free_attrs" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0xb40f8583, "device_create_with_groups" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0xc7c150c3, "seq_read" },
	{ 0xbe76257d, "pv_ops" },
	{ 0x2f6eab6e, "dma_set_coherent_mask" },
	{ 0x15ba50a6, "jiffies" },
	{ 0xe2d5255a, "strcmp" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x2f0beb18, "dma_get_required_mask" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xc37af29f, "param_ops_charp" },
	{ 0xe7f253a, "pci_set_master" },
	{ 0x97934ecf, "del_timer_sync" },
	{ 0x9e5913ff, "pci_alloc_irq_vectors_affinity" },
	{ 0x5f0b34b3, "_dev_warn" },
	{ 0xfb578fc5, "memset" },
	{ 0xdbdf6c92, "ioport_resource" },
	{ 0x1e1e140e, "ns_to_timespec64" },
	{ 0x7bb85bdb, "pci_iounmap" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0xdb05c2fd, "current_task" },
	{ 0x37befc70, "jiffies_to_msecs" },
	{ 0x39992adb, "usb_deregister" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xfef216eb, "_raw_spin_trylock" },
	{ 0x7a017a0a, "sysfs_remove_file_from_group" },
	{ 0x449ad0a7, "memcmp" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0xe186e8b5, "class_unregister" },
	{ 0xde80cd09, "ioremap" },
	{ 0x1edb69d6, "ktime_get_raw_ts64" },
	{ 0x5e7f704c, "usb_set_interface" },
	{ 0x9166fada, "strncpy" },
	{ 0x9afe722, "usb_control_msg" },
	{ 0x7b7ffe44, "pci_read_config_word" },
	{ 0x670ecece, "__x86_indirect_thunk_rbx" },
	{ 0x56e8df72, "dma_alloc_attrs" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0xc38c83b8, "mod_timer" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0xbd120ae1, "__class_register" },
	{ 0x2036ad55, "_dev_err" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x70286235, "pci_find_capability" },
	{ 0x800473f, "__cond_resched" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x167c5967, "print_hex_dump" },
	{ 0xbef4c2c, "i2c_del_adapter" },
	{ 0xd91b158b, "_dev_info" },
	{ 0x94ca3cb5, "usb_submit_urb" },
	{ 0x6383b27c, "__x86_indirect_thunk_rdx" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x142a8383, "usb_reset_device" },
	{ 0x274e1f9d, "usb_bulk_msg" },
	{ 0x1000e51, "schedule" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xa78fd04f, "usb_clear_halt" },
	{ 0x92997ed8, "_printk" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x1035c7c2, "__release_region" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x2ded4fff, "pci_unregister_driver" },
	{ 0xcc5005fe, "msleep_interruptible" },
	{ 0x5f4228d6, "kmem_cache_alloc_trace" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xd8d6e348, "param_ops_byte" },
	{ 0x76ba6d0, "pci_irq_vector" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xf6ebc03b, "net_ratelimit" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0x56b9aa4f, "seq_lseek" },
	{ 0x37a0cba, "kfree" },
	{ 0x69acdf38, "memcpy" },
	{ 0x7ff227c5, "pci_request_regions" },
	{ 0xbe8981ce, "param_array_ops" },
	{ 0xedc03953, "iounmap" },
	{ 0x70c7fc22, "__pci_register_driver" },
	{ 0x7f9d9d6f, "usb_register_driver" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x3a27d0d7, "sysfs_add_file_to_group" },
	{ 0xb0551152, "i2c_bit_add_bus" },
	{ 0x656e4a6e, "snprintf" },
	{ 0xb0e602eb, "memmove" },
	{ 0xfd00e107, "pci_iomap" },
	{ 0x9630c1c3, "param_ops_ushort" },
	{ 0xcbc6deae, "proc_create" },
	{ 0x897b88df, "usb_get_current_frame_number" },
	{ 0x5e515be6, "ktime_get_ts64" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0xd59feaa2, "pci_enable_device" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x13151c68, "param_ops_ulong" },
	{ 0x1867faf3, "param_ops_uint" },
	{ 0xf12cd2a3, "usb_free_urb" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xcde2907a, "usb_alloc_urb" },
	{ 0xc1514a3b, "free_irq" },
};

MODULE_INFO(depends, "i2c-algo-bit");

MODULE_ALIAS("pci:v0000001Cd00000001sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000003sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000004sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000005sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000006sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000007sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000008sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000009sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000002sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000000Asv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000010sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000013sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000014sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000016sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000017sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000018sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000019sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000001Asv*sd*bc*sc*i*");
MODULE_ALIAS("usb:v0C72p000Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p000Dd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0012d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0011d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0013d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0014d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "5CCC3B30E81EBF663BC378A");
