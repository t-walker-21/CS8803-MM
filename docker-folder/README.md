# hackathon_docker
Docker image for hackathon, as well as handy-dandy shell scripts.

## Scripts
Here is an explanation of the scripts we provide, in the order you'll probably use them:

- pull.sh : This file is for pulling image from Dockerhub, it is the first step you should do if you don't have this Docker image locally on your computer. To use it,

    `$ ./pull.sh`

- run.sh : This file is for creating a Docker container named as hackathon_indigo Assume you are still in the hackathon_docker directory,

    `$ ./run.sh`

- connect.sh : This file is for running the Docker container hackathon_indigo. After you execute this, you will be in the container. The color of the characters will change from green to grey on our lab computers.

    `$ ./connect.sh`

- save.sh : This is for saving your work (file changes or setting changes) in your container.

    `$ ./save.sh`

- stop.sh : This is the last step to fully stop the container and remove it (but if you saved it you will not lose data).

    `$ ./stop.sh`

- restart.sh : This is for retrieving your previous work in the saved image,

    `$ ./restart.sh`

Please note that all above commands should be executed on the host computer, not in the container.

## Building

To build:

- `docker build -t mandyxie/hackathon-indigo .`

