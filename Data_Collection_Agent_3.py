import pandas as pd
from Agent_3 import agent_3
from Grid_Generator import gen_grid

df = pd.DataFrame(columns=['Discovered','xi','xj','yi','yj'])

dim = 100

c = 0
x = 0

while c<100:
    grid = gen_grid(dim, 0.3)
    result, final, dis, vis, start, stop, plan_t, bumps, con_vis, df1 = agent_3(grid, dim, 0.3, 2)
    df = pd.concat([df, df1])
    if x%30==0:
        df.to_csv('Agent_3_Data.csv')
        print(c)
    x = x + 1
    c = c + 1
