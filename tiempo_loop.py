import time
# function to test elapsed time for
def print_square(x):
    return x ** 2
i=0
while True:                     
    # records start time
    start = time.time()
    # calls the function
                        
    print_square(3)
    print_square(2)
    print_square(5)
    print_square(315)
    time.sleep(10)
    print_square(8)
    print_square(1)
    if i%2==0:
        time.sleep(7)
    if i%3==0:
        time.sleep(3)
    
    # record end time
    end = time.time()
    
    # find elapsed time in seconds
    ms = (end-start)
    i=i+1
    print(f"Elapsed {ms:.01f} secs.")
    if i==15:
        break                 
