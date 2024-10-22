import requests as r
import sys

level = 19
base_x = 99545 #These are the coordinates for BYU in the tile space
base_y = 198092

tiles_per_10k = 100

print(base_x, base_y)

for i in range(level, 0, -1):
    tiles_at_this_level = int(tiles_per_10k/(2**(level-i)))

    print('level: ' + str(i))

    if tiles_at_this_level == 0:
        answer = r.get('http://localhost:8080/wmts/gm_layer/gm_grid/{}/{}/{}.png'.format(i,base_x,base_y))
        base_x //= 2
        base_y //= 2
        continue

    for j in range(base_x - tiles_at_this_level, base_x + tiles_at_this_level):
        print('row: ' + str(j - base_x))

        for k in range(base_y - tiles_at_this_level, base_y + tiles_at_this_level):
            answer = r.get('http://localhost:8080/wmts/gm_layer/gm_grid/{}/{}/{}.png'.format(i,j,k))
            if (answer.status_code != 200):
                print(answer.status_code)
    base_x //= 2
    base_y //= 2

