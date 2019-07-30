import time
import multiprocessing
from random import random
from functools import partial


def worker(i, arg1, arg2):
    """worker function"""

    # print(arg1)
    # print(arg2)

    print(i)

    time.sleep(0.5)
    
    return random()

if __name__ == '__main__':
    num_workers = multiprocessing.cpu_count()
    with multiprocessing.Pool(num_workers) as pool:
        results = pool.map(partial(worker, arg1=5, arg2=10), range(4))
    
    print(results)
