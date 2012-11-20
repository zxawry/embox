/**
 * @file
 * @brief
 *
 * @date 19.11.12
 * @author Anton Bulychev
 */

#include <errno.h>
#include <assert.h>

#include <mem/misc/pool.h>
#include <kernel/usermode.h>
#include <kernel/thread/api.h>
#include <kernel/thread/sched_lock.h>
#include <kernel/task.h>

#define UE_DATA_POOL_SIZE 5

POOL_DEF(ue_data_pool, struct ue_data, UE_DATA_POOL_SIZE);

static void usermode_trampoline(struct ue_data *data) {
	struct ue_data s_data;

	sched_lock();
	{
		s_data = (struct ue_data) {
			.ip = data->ip,
			.sp = data->sp,
		};
		pool_free(&ue_data_pool, data);
	}
	sched_unlock();

	usermode_entry(&s_data);
}

#define TRAMPOLINE ((void * (*)(void *)) usermode_trampoline)

int create_usermode_thread(struct thread **t, unsigned int flags,
		void *ip, void *sp) {
	struct ue_data *data;
	int err;

	sched_lock();
	{
		if (!(data = pool_alloc(&ue_data_pool))) {
			sched_unlock();
			return -EAGAIN;
		}

		data->ip = ip;
		data->sp = sp;

		if ((err = thread_create(t, flags, TRAMPOLINE, data))) {
			pool_free(&ue_data_pool, data);
			sched_unlock();
			return err;
		}
	}
	sched_unlock();

	return ENOERR;
}

int create_usermode_task(void *ip, void *sp) {
	struct ue_data *data;
	int err;

	sched_lock();
	{
		if (!(data = pool_alloc(&ue_data_pool))) {
			sched_unlock();
			return -EAGAIN;
		}

		data->ip = ip;
		data->sp = sp;

		if ((err = new_task(TRAMPOLINE, data, 0)) < 0) {
			pool_free(&ue_data_pool, data);
			sched_unlock();
			return err;
		}
	}
	sched_unlock();

	return err;
}

