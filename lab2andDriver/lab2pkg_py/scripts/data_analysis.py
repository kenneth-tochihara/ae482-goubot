import numpy as np
import matplotlib.pyplot as plt

# check for float function
def isfloat(x):
    try:
        a = float(x)
    except (TypeError, ValueError):
        return False
    else:
        return True

# open file and read contents
with open('data.csv', 'r') as f:
    data = f.readlines()
    
# parse for headers
headers = data[0].split(',')

# parse for data entry
parsed_data = []
for line in data[1:]:
    line = line.strip().split(',')
    
    entry_data = []
    for idx, header in enumerate(line):
        
        # check for int
        if line[idx].isnumeric():
            entry_data.append(int(line[idx]))
        
        # check for float
        elif isfloat(line[idx]):
            entry_data.append(float(line[idx]))
        
        else:
            entry_data.append("N/A")
    
    parsed_data.append(dict(zip(headers, entry_data)))

# graph of xy plane
markers = ['X', 'o']
colors = ['red', 'green']
fig = plt.figure()
for line in parsed_data:
    plt.scatter(line['block x'], line['block y'], marker=markers[line['success on first try?']], c=colors[line['success on first try?']])
plt.scatter(None, None, marker=markers[1], c=colors[1], label='Success')
plt.scatter(None, None, marker=markers[0], c=colors[0], label='Failure')
plt.xlabel(r'$x$ (m)')
plt.ylabel(r'$y$ (m)')
plt.axis('equal')
plt.grid(); plt.tight_layout(); plt.legend()
plt.show()
fig.clear()

# table of -x vs +x success rates
positive = 0; positive_cnt = 0
negative = 0; negative_cnt = 0

for line in parsed_data:
    if line['block x'] < 0:
        negative += line['success on first try?']
        negative_cnt += 1
    else:
        positive += line['success on first try?']
        positive_cnt += 1
        
print(f'negative success = {negative/negative_cnt}')
print(f'positive success = {positive/positive_cnt}')

# plot the error values
for line in parsed_data:
    if line['dx'] == "N/A": continue
    plt.scatter(line['dx'], line['dy'], c='blue')
plt.xlabel(r'$e_x$ (m)')
plt.ylabel(r'$e_y$ (m)')
plt.axis('equal')
plt.grid(); plt.tight_layout()
plt.show()