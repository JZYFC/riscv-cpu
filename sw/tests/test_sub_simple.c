/* Minimal SUB test — no function calls, no loops, minimum branches.
 * If this fails: sub instruction or store-load ordering bug.
 * If this passes but test_add fails at code 3: register rename recovery bug.
 */
__attribute__((noinline)) int sub_func(int a, int b) { return a - b; }

int main(void) {
    volatile int a, b, r;

    /* Test 1: direct subtraction, no function call */
    a = 10; b = 3;
    r = a - b;
    if (r != 7) return 1;

    /* Test 2: subtraction via function call */
    a = 10; b = 3;
    r = sub_func(a, b);
    if (r != 7) return 2;

    /* Test 3: sub immediately after add to same volatile */
    a = 5; b = 3;
    r = a + b;           /* r = 8 */
    if (r != 8) return 3;
    r = a - b;           /* r = 2, overwrites same volatile slot */
    if (r != 2) return 4;

    return 0;
}
