'''
    计时器
'''

import time

def timer(func):
    def func_in(*args, **kwargs):
        start_time = time.time()
        func_res = func(*args, **kwargs)
        end_time = time.time()
        spend_time = (end_time - start_time)
        print(f"Function '{func.__name__}' took {spend_time:.6f} seconds to execute.")
        return func_res
    return func_in