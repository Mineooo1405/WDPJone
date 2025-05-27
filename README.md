python version in used 3.12.4 64bit

Step 1: install frontend 

    for linux: sudo apt install nodejs npm
    cd ./front
    npm start

Step 2: install python libraries

    $ "path to python3.12.4" -m venv .venv
    source ".env/bin/activate"
    pip install -r requirements.txt

Step 3: run project

    cd ./back
    python start.py
    click start all 

Step 4: test with simulate data

    cd ./back
    python simulate_robots.py
    json package frequency can be set in these variables
            encoder_interval 
            imu_interval     
            log_interval 
    

