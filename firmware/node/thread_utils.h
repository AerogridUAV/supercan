#ifndef THREAD_UTILS_H
#define THREAD_UTILS_H

#include <ch.h>
#include <hal.h>

/**
 * @brief Create a thread with dynamic memory allocation
 * @param stack_size Stack size in bytes
 * @param name Thread name (for debugging)
 * @param priority Thread priority
 * @param func Thread function
 * @param arg Thread argument
 * @return Pointer to created thread or NULL on failure
 */
static inline thread_t *create_dynamic_thread(const char *name, size_t stack_size,
                                              tprio_t priority, tfunc_t func, void *arg) {
    thread_t *thd = chThdCreateFromHeap(NULL, stack_size, name, priority, func, arg);
    if (thd == NULL) {
        // Could add logging here if needed
        // chprintf((BaseSequentialStream*)&SD1, "Failed to create thread: %s\r\n", name);
    }
    return thd;
}

/**
 * @brief Macro to simplify dynamic thread creation
 */
#define CREATE_DYNAMIC_THREAD(name, stack_size, priority, func, arg) \
    create_dynamic_thread(name, stack_size, priority, func, arg)

#endif /* THREAD_UTILS_H */
