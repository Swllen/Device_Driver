# 放在最顶端、在 import numpy 之前！！！
import os
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"

import time
import numpy as np
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor, as_completed
from functools import partial
import multiprocessing as mp

def matmul_task(size: int, repeat: int = 20):
    # 只做乘法，避免把随机数生成时间算进去
    A = np.random.rand(size, size)
    B = np.random.rand(size, size)
    # 热身一次，避免把初始化成本算在计时里
    _ = A @ B
    s = 0.0
    t0 = time.perf_counter()
    print(f"开始任务: {size}x{size} 矩阵乘法，重复 {repeat} 次")
    for _ in range(repeat):
        s += (A @ B).sum()
    return s, time.perf_counter() - t0

def run_parallel(funcs, executor_cls):
    t0 = time.perf_counter()
    results = {}
    with executor_cls(max_workers=min(len(funcs), mp.cpu_count())) as ex:
        futs = {ex.submit(f): name for name, f in funcs.items()}
        for fut in as_completed(futs):
            results[futs[fut]] = fut.result()
    return results, time.perf_counter() - t0

if __name__ == "__main__":
    # 任务规模：按你的机器调 size/repeat；小了看不出差异，大了会很慢
    size = 1200
    repeat = 5
    funcs = {
        f"task{i}": partial(matmul_task, size, repeat)
        for i in range(1, 1 + min(4, mp.cpu_count()))   # 任务数≈CPU核数
    }

    # 串行
    t0 = time.perf_counter()
    serial_res = {k: f() for k, f in funcs.items()}
    t_serial = time.perf_counter() - t0
    print("串行耗时:", round(t_serial, 3), "s")

    # 进程并行（CPU密集型推荐）
    par_res_p, t_par_p = run_parallel(funcs, ProcessPoolExecutor)
    print("进程池并发耗时:", round(t_par_p, 3), "s", "  加速比:", round(t_serial / t_par_p, 2))

    # 线程并行（在已将 BLAS 线程=1 时，有时也能有收益；视平台而定）
    results,_ = par_res_t, t_par_t = run_parallel(funcs, ThreadPoolExecutor)
    print("线程池结果:", results)
    print("线程池并发耗时:", round(t_par_t, 3), "s", "  加速比:", round(t_serial / t_par_t, 2))
