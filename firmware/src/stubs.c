#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

void exit(int status) {
    printk("exit() called with status %d\n", status);
    while (1) {
        k_sleep(K_FOREVER);
    }
}

int __aeabi_f2iz(float x) {
    return (int)x;
}

int rand(void) {
    return (int)sys_rand32_get();
}

void srand(unsigned int seed) {
    (void)seed;
}

unsigned long strtoul(const char *nptr, char **endptr, int base) {
    (void)nptr; (void)endptr; (void)base;
    return 0;
}

size_t fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream) {
    (void)stream;
    const char * cptr = (const char *)ptr;
    for (size_t i = 0; i < size * nmemb; i++) {
        printk("%c", cptr[i]);
    }
    return nmemb;
}

int vsnprintf(char *str, size_t size, const char *format, va_list ap) {
    return vsnprintk(str, size, format, ap);
}

int memcmp(const void *s1, const void *s2, size_t n) {
    const unsigned char *p1 = s1, *p2 = s2;
    while(n--) {
        if (*p1 != *p2) return *p1 - *p2;
        p1++; p2++;
    }
    return 0;
}

char *strstr(const char *haystack, const char *needle) {
    if (!*needle) return (char *)haystack;
    const char *p1, *p2;
    while (*haystack) {
        p1 = haystack; p2 = needle;
        while (*p1 && *p2 && *p1 == *p2) { p1++; p2++; }
        if (!*p2) return (char *)haystack;
        haystack++;
    }
    return NULL;
}
