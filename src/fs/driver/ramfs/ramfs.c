/**
 * @file
 * @brief Tmp file system
 *
 * @date 12.11.12
 * @author Andrey Gazukin
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include <fs/fs_driver.h>
#include <fs/vfs.h>
#include <fs/ramfs.h>
#include <util/array.h>
#include <embox/unit.h>
#include <embox/block_dev.h>
#include <mem/misc/pool.h>
#include <mem/phymem.h>
#include <drivers/ramdisk.h>
#include <fs/file_system.h>
#include <fs/file_desc.h>
#include <limits.h>

/* ramfs filesystem description pool */
POOL_DEF(ramfs_fs_pool, struct ramfs_fs_info, OPTION_GET(NUMBER,ramfs_descriptor_quantity));

/* ramfs file description pool */
POOL_DEF(ramfs_file_pool, struct ramfs_file_info, OPTION_GET(NUMBER,inode_quantity));

INDEX_DEF(ramfs_file_idx,0,OPTION_GET(NUMBER,inode_quantity));

/* define sizes in 4096 blocks */
#define MAX_FILE_SIZE OPTION_GET(NUMBER,ramfs_file_size)
#define FILESYSTEM_SIZE OPTION_GET(NUMBER,ramfs_filesystem_size)

#define RAMFS_NAME "ramfs"
#define RAMFS_DEV  "/dev/ram#"
#define RAMFS_DIR  "/"

static char sector_buff[PAGE_SIZE()];/* TODO */

static int ramfs_format(void *path);
static int ramfs_mount(void *dev, void *dir);

static int ramfs_init(void * par) {
	struct node *dev_node, *dir_node;
	int res;

	if (!par) {
		return 0;
	}

	/*TODO */

	dir_node = vfs_lookup(NULL, RAMFS_DIR);
	if (!dir_node) {
		return -1;
	}

	if (0 != (res = ramdisk_create(RAMFS_DEV, FILESYSTEM_SIZE * PAGE_SIZE()))) {
		return res;
	}

	dev_node = vfs_lookup(NULL, RAMFS_DEV);
	if (!dev_node) {
		return -1;
	}

	/* format filesystem */
	if (0 != (res = ramfs_format((void *) dev_node))) {
		return res;
	}

	/* mount filesystem */
	return ramfs_mount(dev_node, dir_node);
}

static int ramfs_ramdisk_fs_init(void) {
	return ramfs_init(RAMFS_DEV);
}

EMBOX_UNIT_INIT(ramfs_ramdisk_fs_init); /*TODO*/


static int    ramfs_open(struct node *node, struct file_desc *file_desc, int flags);
static int    ramfs_close(struct file_desc *desc);
static size_t ramfs_read(struct file_desc *desc, void *buf, size_t size);
static size_t ramfs_write(struct file_desc *desc, void *buf, size_t size);
static int    ramfs_ioctl(struct file_desc *desc, int request, ...);

static struct kfile_operations ramfs_fop = {
	.open = ramfs_open,
	.close = ramfs_close,
	.read = ramfs_read,
	.write = ramfs_write,
	.ioctl = ramfs_ioctl,
};

/*
 * file_operation
 */

static int ramfs_open(struct node *node, struct file_desc *desc, int flags) {
	struct nas *nas;
	ramfs_file_info_t *fi;

	nas = node->nas;
	fi = (ramfs_file_info_t *)nas->fi->privdata;

	fi->pointer = desc->cursor;

	return 0;
}

static int ramfs_close(struct file_desc *desc) {
	return 0;
}

static int ramfs_read_sector(struct nas *nas, char *buffer,
		uint32_t count, uint32_t sector) {
	struct ramfs_fs_info *fsi;

	fsi = nas->fs->fsi;

	if(0 > block_dev_read(nas->fs->bdev, (char *) buffer,
			count * fsi->block_size, sector)) {
		return -1;
	}
	else {
		return count;
	}
}

static int ramfs_write_sector(struct nas *nas, char *buffer,
		uint32_t count, uint32_t sector) {
	struct ramfs_fs_info *fsi;

	fsi = nas->fs->fsi;

	if(0 > block_dev_write(nas->fs->bdev, (char *) buffer,
			count * fsi->block_size, sector)) {
		return -1;
	}
	else {
		return count;
	}
}

static size_t ramfs_read(struct file_desc *desc, void *buf, size_t size) {
	size_t len;
	size_t current, cnt;
	uint32_t end_pointer;
	blkno_t blk;
	uint32_t bytecount;
	uint32_t start_block;		/* start of file */
	struct nas *nas;
	struct ramfs_fs_info *fsi;
	struct ramfs_file_info *fi;

	nas = desc->node->nas;
	fi = nas->fi->privdata;
	fsi = nas->fs->fsi;

	fi->pointer = desc->cursor;

	len = size;

	/* Don't try to read past EOF */
	if (len > nas->fi->ni.size - fi->pointer) {
		len = nas->fi->ni.size - fi->pointer;
	}

	end_pointer = fi->pointer + len;
	bytecount = 0;
	start_block = fi->index * fsi->block_per_file;

	while(len) {
		blk = fi->pointer / fsi->block_size;
		/* check if block over the file */
		if(blk >= fsi->block_per_file) {
			bytecount = 0;
			break;
		}
		else {
			blk += start_block;
		}
		/* check if block over the filesystem */
		if(blk >= fsi->numblocks) {
			bytecount = 0;
			break;
		}
		/* calculate pointer in scratch buffer */
		current = fi->pointer % fsi->block_size;

		/* set the counter how many bytes read from block */
		if(end_pointer - fi->pointer > fsi->block_size) {
			if(current) {
				cnt = fsi->block_size - current;
			}
			else {
				cnt = fsi->block_size;
			}
		}
		else {
			cnt = end_pointer - fi->pointer;
		}

		/* one 4096-bytes block read operation */
		if(1 != ramfs_read_sector(nas, sector_buff, 1, blk)) {
			bytecount = 0;
			break;
		}
		/* get data from block */
		memcpy (buf, sector_buff + current, cnt);

		bytecount += cnt;
		/* shift the pointer */
		fi->pointer += cnt;
		if(end_pointer >= fi->pointer) {
			break;
		}
	}

	desc->cursor = fi->pointer;
	return bytecount;
}


static size_t ramfs_write(struct file_desc *desc, void *buf, size_t size) {
	ramfs_file_info_t *fi;
	size_t len;
	size_t current, cnt;
	uint32_t end_pointer;
	blkno_t blk;
	uint32_t bytecount;
	uint32_t start_block;
	struct nas *nas;
	struct ramfs_fs_info *fsi;

	nas = desc->node->nas;
	fi = (ramfs_file_info_t *)nas->fi->privdata;
	fsi = nas->fs->fsi;

	bytecount = 0;

	fi->pointer = desc->cursor;
	len = size;
	end_pointer = fi->pointer + len;
	start_block = fi->index * fsi->block_per_file;

	while(1) {
		if(0 == fsi->block_size) {
			break;
		}
		blk = fi->pointer / fsi->block_size;
		/* check if block over the file */
		if(blk >= fsi->block_per_file) {
			bytecount = 0;
			break;
		}
		else {
			blk += start_block;
		}
		/* calculate pointer in scratch buffer */
		current = fi->pointer % fsi->block_size;

		/* set the counter how many bytes written in block */
		if(end_pointer - fi->pointer > fsi->block_size) {
			if(current) {
				cnt = fsi->block_size - current;
			}
			else {
				cnt = fsi->block_size;
			}
		}
		else {
			cnt = end_pointer - fi->pointer;
			/* over the block ? */
			if((current + cnt) > fsi->block_size) {
				cnt -= (current + cnt) % fsi->block_size;
			}
		}

		/* one 4096-bytes block read operation */
		if(1 != ramfs_read_sector(nas, sector_buff, 1, blk)) {
			bytecount = 0;
			break;
		}
		/* set new data in block */
		memcpy (sector_buff + current, buf, cnt);

		/* write one block to device */
		if(1 != ramfs_write_sector(nas, sector_buff, 1, blk)) {
			bytecount = 0;
			break;
		}
		bytecount += cnt;
		/* shift the pointer */
		fi->pointer += cnt;
		if(end_pointer <= fi->pointer) {
			break;
		}
	}
	/* if we write over the last EOF, set new filelen */
	if (nas->fi->ni.size < fi->pointer) {
		nas->fi->ni.size = fi->pointer;
	}

	desc->cursor = fi->pointer;
	return bytecount;
}

static int ramfs_ioctl(struct file_desc *desc, int request, ...) {
	return 0;
}

/*
static int ramfs_seek(void *file, long offset, int whence);
static int ramfs_stat(void *file, void *buff);

static int ramfs_fseek(void *file, long offset, int whence) {
	struct file_desc *desc;
	ramfs_file_info_t *fi;
	uint32_t curr_offset;

	curr_offset = offset;

	desc = (struct file_desc *) file;
	fi = (ramfs_file_info_t *)desc->node->fi;

	switch (whence) {
	case SEEK_SET:
		break;
	case SEEK_CUR:
		curr_offset += fi->pointer;
		break;
	case SEEK_END:
		curr_offset = fi->filelen + offset;
		break;
	default:
		return -1;
	}

	if (curr_offset > fi->filelen) {
		curr_offset = fi->filelen;
	}
	fi->pointer = curr_offset;
	return 0;
}
static int ramfs_stat(void *file, void *buff) {
	struct file_desc *desc;
	ramfs_file_info_t *fi;
	stat_t *buffer;

	desc = (struct file_desc *) file;
	fi = (ramfs_file_info_t *)desc->node->fi;
	buffer = (stat_t *) buff;

	if (buffer) {
			memset(buffer, 0, sizeof(stat_t));

			buffer->st_mode = fi->mode;
			buffer->st_ino = fi->index;
			buffer->st_nlink = 1;
			buffer->st_dev = *(int *) fsi->bdev;
			buffer->st_atime = buffer->st_mtime = buffer->st_ctime = 0;
			buffer->st_size = fi->filelen;
			buffer->st_blksize = fsi->block_size;
			buffer->st_blocks = fsi->numblocks;
		}

	return fi->filelen;
}
*/


static int ramfs_init(void * par);
static int ramfs_format(void *path);
static int ramfs_mount(void *dev, void *dir);
static int ramfs_create(struct node *parent_node, struct node *node);
static int ramfs_delete(struct node *node);
static int ramfs_truncate(struct node *node, off_t length);

static struct fsop_desc ramfs_fsop = {
	.init = ramfs_init,
	.format = ramfs_format,
	.mount = ramfs_mount,
	.create_node = ramfs_create,
	.delete_node = ramfs_delete,

	.truncate = ramfs_truncate,
};

static struct fs_driver ramfs_driver = {
	.name = RAMFS_NAME,
	.file_op = &ramfs_fop,
	.fsop = &ramfs_fsop,
};

static ramfs_file_info_t *ramfs_create_file(struct nas *nas) {
	ramfs_file_info_t *fi;

	fi = pool_alloc(&ramfs_file_pool);
	if (!fi) {
		return NULL;
	}

	fi->index = index_alloc(&ramfs_file_idx, INDEX_ALLOC_MIN);
	nas->fi->ni.size = fi->pointer = 0;

	return fi;
}

/*
static node_t *ramfs_create_dot(node_t *parent_node, const char *name) {
	node_t *dot_node;
	struct nas *parent_nas, *nas;

	parent_nas = parent_node->nas;

	dot_node = vfs_create_child(parent_node, name, S_IFDIR);
	if (dot_node) {
		nas = dot_node->nas;
		nas->fs = parent_nas->fs;
		// don't need create fi for directory - take root node fi /
		nas->fi->privdata = parent_nas->fi->privdata;
	}

	return dot_node;
}
*/

static int ramfs_create(struct node *parent_node, struct node *node) {
	struct nas *nas;

	nas = node->nas;

	if (!node_is_directory(node)) {
		if (!(nas->fi->privdata = ramfs_create_file(nas))) {
			return -ENOMEM;
		}
	}

	nas->fs = parent_node->nas->fs;

	return 0;
}

static int ramfs_delete(struct node *node) {
	struct ramfs_file_info *fi;
	struct ramfs_fs_info *fsi;
	struct nas *nas;
	char path [PATH_MAX];

	nas = node->nas;
	fi = nas->fi->privdata;
	fsi = nas->fs->fsi;

	vfs_get_path_by_node(node, path);

	if (!node_is_directory(node)) {
		index_free(&ramfs_file_idx, fi->index);
		pool_free(&ramfs_file_pool, fi);
	}

	/* root node - have fi, but haven't index*/
	if(0 == strcmp((const char *) path, (const char *) nas->fs->mntto)){
		pool_free(&ramfs_fs_pool, fsi);
	}

	vfs_del_leaf(node);

	return 0;
}

static int ramfs_truncate(struct node *node, off_t length) {
	struct nas *nas = node->nas;

	if (length > MAX_FILE_SIZE) {
		return -EFBIG;
	}

	nas->fi->ni.size = length;

	return 0;
}

static int ramfs_format(void *dev) {
	node_t *dev_node;
	struct nas *dev_nas;
	struct node_fi *dev_fi;

	if (NULL == (dev_node = dev)) {
		return -ENODEV;/*device not found*/
	}

	if(!node_is_block_dev(dev_node)) {
		return -ENODEV;
	}
	dev_nas = dev_node->nas;
	dev_fi = dev_nas->fi;

	if(MAX_FILE_SIZE > block_dev(dev_fi->privdata)->size / PAGE_SIZE()) {
		return -ENOSPC;
	}
	return 0;
}

static int ramfs_mount(void *dev, void *dir) {
	struct node *dir_node, *dev_node;
	struct nas *dir_nas, *dev_nas;
	struct ramfs_file_info *fi;
	struct ramfs_fs_info *fsi;
	struct node_fi *dev_fi;

	dev_node = dev;
	dev_nas = dev_node->nas;
	dir_node = dir;
	dir_nas = dir_node->nas;

	if (NULL == (dev_fi = dev_nas->fi)) {
		return -ENODEV;
	}

	if (NULL == (dir_nas->fs = filesystem_alloc("ramfs"))) {
		return -ENOMEM;
	}
	dir_nas->fs->bdev = dev_fi->privdata;

	/* allocate this fs info */
	if(NULL == (fsi = pool_alloc(&ramfs_fs_pool))) {
		filesystem_free(dir_nas->fs);
		return -ENOMEM;
	}
	memset(fsi, 0, sizeof(struct ramfs_fs_info));
	dir_nas->fs->fsi = fsi;
	vfs_get_path_by_node(dir_node, dir_nas->fs->mntto);
	vfs_get_path_by_node(dev_node, dir_nas->fs->mntfrom);
	fsi->block_per_file = MAX_FILE_SIZE;
	fsi->block_size = PAGE_SIZE();
	fsi->numblocks = block_dev(dev_fi->privdata)->size / PAGE_SIZE();

	/* allocate this directory info */
	if(NULL == (fi = pool_alloc(&ramfs_file_pool))) {
		return -ENOMEM;
	}
	memset(fi, 0, sizeof(struct ramfs_file_info));
	fi->index = fi->mode = 0;
	fi->pointer = 0;
	dir_nas->fi->privdata = (void *) fi;

	return 0;
}

DECLARE_FILE_SYSTEM_DRIVER(ramfs_driver);

