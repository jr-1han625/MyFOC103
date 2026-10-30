/* L4 驱动静态装配入口：启动时先调用，再使用各 L1 B 接口。
 * 只绑定能力；实际启用哪些器件由 main 的初始化顺序决定。
 */
#ifndef DRIVER_ASSEMBLY_H
#define DRIVER_ASSEMBLY_H
void Drivers_Bind(void);
#endif
