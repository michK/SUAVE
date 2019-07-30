import time
import multiprocessing
from numpy.random import random

def worker(i, return_dict):
    """worker function"""
    print('Worker {} complete'.format(i))
    return_dict[i] = random()
    time.sleep(1)
    
    return

if __name__ == '__main__':
    jobs = []
    num_workers = multiprocessing.cpu_count()
    # pool = multiprocessing.Pool(num_workers)
    for i in range(num_workers):
        manager = multiprocessing.Manager()
        return_dict = manager.dict()
        p = multiprocessing.Process(target=worker, args=(i, return_dict))
        jobs.append(p)
        p.start()
        p.join()

    print(return_dict.values())
