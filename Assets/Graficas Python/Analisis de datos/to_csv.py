import csv
import os

FILE = "data.log"
OUT = "data.csv"

data = []
clean = []
with open(FILE,"+rt") as f:
    data = f.readlines()
for line in data:
    if line == "\n":
        continue
    rip = f"{line[1:-2]}\n"
    clean.append(rip)

with open(OUT,"wt") as f:
    f.writelines(clean)





