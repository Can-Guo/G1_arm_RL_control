Arm Control Protocol from MCB Calling to Robot.
日期: 20251104

描述：from MCB to robot control for arm.


Arm.action

名稱：action_arm

參數Table：


    ID	數據類型	參數名	參數
    

    1.目標Goal

    1	string	target	# empty_box/product_box
    2	string	action	# pick/place/move.  pick:  執行pick動作; place: 執行放置動作; move: 執行移動動作
    3	float64	x_l	
    4	float64	y_l	
    5	float64	z_l	
    6	float64	x_r	
    7	float64	y_r	
    8	float64	z_r	



    2. 反饋Feedback:

        1	float64	x_l	
        2	float64	y_l	
        3	float64	z_l	
        4	float64	x_r	
        5	float64	y_r	
        6	float64	z_r	



    3. 結果Result:	

        1	string	result	# success/failed
        2	string	error	# error info
        3	float64	x_l	
        4	float64	y_l	
        5	float64	z_l	
        6	float64	x_r	
        7	float64	y_r	
        8	float64	z_r	

