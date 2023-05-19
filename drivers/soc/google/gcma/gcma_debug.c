#include <linux/types.h>
#include <linux/debugfs.h>

static bool workingset = true;

bool workingset_filter_enabled(void)
{
	return workingset;
}

int __init gcma_debugfs_init(void)
{
	struct dentry *gcma_debugfs_root;

	gcma_debugfs_root = debugfs_create_dir("gcma", NULL);
	if (!gcma_debugfs_root)
		return -ENOMEM;

	debugfs_create_bool("workingset", 0644, gcma_debugfs_root, &workingset);

	return 0;
}
