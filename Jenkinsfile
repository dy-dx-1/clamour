#!groovy

pipeline {
    agent any

    environment {
        CONTAINER_NAME = "clamour"
        CONTAINER_TAG = "latest"
        DOCKER_HUB_CREDENTIALS = credentials("docker_hub")
    }

    stages {
        stage("Installs / Updates") {
            steps {
                // The 'unittest' package is not included in the requirements.txt, 
                // because it is only required for testing and should not be included in the image.
                sh "/usr/bin/pip3 install unittest"
                sh "/usr/bin/pip3 install -r requirements.txt"
            }
        }

        stage("Tests") {
            steps {
                script {
                    rc = sh(script: "python3 tests/tests.ut.py -v", returnStatus: true)

                    if(rc != 0) {
                        error("Tests failed")
                    }
                }
            }
        }

        stage("Deployment") {
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
