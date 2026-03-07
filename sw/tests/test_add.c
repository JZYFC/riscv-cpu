/* Basic arithmetic test for OoO RV32I CPU
 *
 * Uses volatile to prevent the compiler from folding everything at compile
 * time, ensuring real instructions are executed on the CPU.
 *
 * Tests: add, sub, loop accumulation, nested calls.
 * Returns: 0 = PASS, nonzero = FAIL (code indicates which check failed).
 *
 * Compile with: -march=rv32i -mabi=ilp32 -nostdlib -O2
 */

/* Prevent compiler from seeing through function bodies */
__attribute__((noinline)) int add(int a, int b) { return a + b; }
__attribute__((noinline)) int sub(int a, int b) { return a - b; }

int main(void) {
    volatile int a, b, r;

    /* Test 1: 3 + 4 = 7 */
    a = 3; b = 4;
    r = add(a, b);
    if (r != 7)  return 1;

    /* Test 2: -1 + 1 = 0 */
    a = -1; b = 1;
    r = add(a, b);
    if (r != 0)  return 2;

    /* Test 3: 10 - 3 = 7 */
    a = 10; b = 3;
    r = sub(a, b);
    if (r != 7)  return 3;

    /* Test 4: sum 1..10 = 55 */
    volatile int sum = 0;
    volatile int i;
    for (i = 1; i <= 10; i++)
        sum = add(sum, i);
    if (sum != 55) return 4;

    /* Test 5: add(sub(8,3), add(2,2)) = 9 */
    r = add(sub(8, 3), add(2, 2));
    if (r != 9)  return 5;

    return 0;  /* PASS */
}
