pipeline {
    agent any

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
                buildImage("clamour", "latest")
                pushImage("clamour", "latest", "samsei", "pz9sQ9tUMQy5ZFU")
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
