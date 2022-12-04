# import pandas lib as pd
import numpy as np
import pandas as pd


class acc_directory():
    acceleration.x
    acceleration.y
    acceleration.z


def relevant_columns():
    fields = ['roll', 'pitch']

    df = pd.read_csv('2022-11-27 15-13-48 vehicle1.csv', skipinitialspace=True, usecols=fields)
    #print(df.roll)
    #print(df.pitch)
    print(df)
    mat = np.matrix(df)
    with open('relevant_columns', 'wb') as f:
        for line in mat:
            np.savetxt(f, line, fmt='%.5f')

def determine_directory(): 
    #if()


if __name__ == "__main__":
    relevant_columns()
    #acc_directory()
    #determine_directory() 