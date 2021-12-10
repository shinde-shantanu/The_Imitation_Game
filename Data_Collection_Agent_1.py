import pandas as pd
from Agent_1 import repeated_a_star
from Grid_Generator import gen_grid

df = pd.DataFrame(columns=['Discovered','xi','xj','yi','yj'])

dim = 100

c = 0
x = 0

while c<10000:
    grid = gen_grid(dim, 0.3)
    result, final, dis, cells, start, stop, df1 = repeated_a_star(grid, dim, 0.3, 2)
    df = pd.concat([df, df1])
    if x%30==0:
        df.to_csv('Agent_1_Data.csv')
        print(c)
    x = x + 1
    c = c + 1
