import os

try:
    with open('sim_output_2.txt', 'r', encoding='utf-16') as f:
        print(f.read())
except UnicodeError:
    try:
        with open('sim_output_2.txt', 'r', encoding='utf-8') as f:
            print(f.read())
    except UnicodeError:
        with open('sim_output_2.txt', 'rb') as f:
            print(f.read().decode('latin-1'))
except Exception as e:
    print(f"Error reading log: {e}")
