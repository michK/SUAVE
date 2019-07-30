import time
import multiprocessing
from numpy.random import random
from itertools import repeat
from functools import partial

def worker(i, args):
    """worker function"""
    a, b = args
    print('Worker {}: {} + {}'.format(i, a, b))
    time.sleep(1)

    res = i**2
    
    return res

if __name__ == '__main__':
    num_workers = multiprocessing.cpu_count()
    args = [1,2]
    with multiprocessing.Pool(num_workers) as pool:
            # results = pool.map(worker, range(4))
            # results = pool.starmap(worker, zip(range(10), repeat(args)))
            # results = pool.map(partial(worker, args), range(10))
            results = pool.starmap(worker, zip(range(10), repeat(args)))

    print(results)
