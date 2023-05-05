my_list = ['a', 'b', 'c', 'd', 'e', 'f']
specific_element = 'd'

# find the index of the specific element
index = my_list.index(specific_element)

# slice the list to get the elements before the index,
# put them at the end of the list in order
new_list = my_list[index:] + my_list[:index]

print(new_list)