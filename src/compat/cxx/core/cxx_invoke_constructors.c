/*
 * @file
 *
 * @date Sep 11, 2012
 * @author: Anton Bondarev
 */

typedef void (*ctor_func_t)(void);

__attribute__((weak)) void __register_frame(void *begin) {
}

static int register_eh_frame(void) {
	extern const char __EH_FRAME_BEGIN__;
	__register_frame((void *)&__EH_FRAME_BEGIN__);
	return 0;
}

void cxx_invoke_constructors(void) {
	extern const char _ctors_start, _ctors_end;
	ctor_func_t *func;
	int n_entries;

	/* Required only if .eh_frame is used for stack unwinding,
	 * for example, on x86. On arm .ARM.exidx is used, so we this function
	 * will be empty. */
	register_eh_frame();

	for (func = (ctor_func_t *) &_ctors_start, n_entries = 0;
			*func && (func != (ctor_func_t *) &_ctors_end);
			func++, n_entries++) {
				(*func)();
	}
}
