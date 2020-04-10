To use the autograder, do the following:

1) Ensure that your MATLAB path contains both the autograder folder and whatever directory your function is saved in. For this autograder, it will look for functions named "calcJacobian_*.m" where the * is a placeholder for any text.

2) Call "testJacobian" without any arguments. It will run tests on your scripts and provide you with (hopefully) meaningful feedback in the command window. Your final score will be output as a MATLAB table.

3) DO NOT call any of the other functions in the autograder directory. They are not designed to be called independently and may throw errors. Run only "testJacobian," and it will manage calls to all other functions.

4) If you have multiple versions of your function in your directory, testJacobian will test them all in sequence. You may optionally call "testJacobian(mygroupno)" with the argument being the value (as a string) of the * in (1) for which you want the output. The command window will show the output for all the functions, but the output of testJacobian will be based on the parameter.