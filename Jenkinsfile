#!groovy

pipeline {
    agent any

    stages {
        stage("Tests") {
            steps {
                script {
                    echo "Skipping tests for now"
                    /*rc = sh(script: "python3 tests/main.ut.py -v", returnStatus: true)

                    if(rc != 0) {
                        error("Tests failed")
                    }*/
                }
            }
        }

        stage("Deployment") {
             environment {
                CONTAINER_NAME = "clamour"
                CONTAINER_TAG = "latest"
                DOCKER_HUB_CREDENTIALS = credentials("docker_hub")
            }
            
            steps {
                buildImage(CONTAINER_NAME, CONTAINER_TAG)
                pushImage(CONTAINER_NAME, CONTAINER_TAG, DOCKER_HUB_CREDENTIALS_USR, DOCKER_HUB_CREDENTIALS_PSW)
            }
        }
    }

    post {
        always {
            cleanWs()
        }
    }
}

def buildImage(containerName, tag) {
    sh "docker build -t ${containerName}:${tag} ."
}

def pushImage(containerName, tag, dockerUser, dockerPassword) {
    sh "docker login -u ${dockerUser} -p ${dockerPassword}"
    sh "docker tag ${containerName}:${tag} ${dockerUser}/${containerName}:${tag}"
    sh "docker push ${dockerUser}/${containerName}:${tag}"
    echo "Image push complete"
}
