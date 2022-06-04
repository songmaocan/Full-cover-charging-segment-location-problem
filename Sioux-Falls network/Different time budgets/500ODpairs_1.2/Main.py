"""
Algorithm: Solve the  wireless charging full covering model by CG
Copyright: Maocan Song, 1097133316@qq.com
Date: 2022-4-20
"""
import time
from Method import Solve

def main():
    start_time = time.time()
    mod=Solve()
    mod.g_solving_FCCSLP_by_CG()
    end_time = time.time()
    spend_time = end_time - start_time
    mod.output_results(spend_time)
    print("CPU running time {} min".format(round(spend_time / 60), 3))

if __name__ == '__main__':
    main()