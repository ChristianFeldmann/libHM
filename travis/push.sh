#!/bin/sh

setup_git() {
  git config --global user.email "christian.feldmann@gmx.de"
  git config --global user.name "ChristianFeldmann"
}

commit_website_files() {
  git checkout -b upstreamFiles
  git add test.txt
  git commit --message "Travis build: $TRAVIS_BUILD_NUMBER"
}

upload_files() {
  git remote add compiledLibraries https://${GH_TOKEN}@github.com/ChristianFeldmann/libHM.git > /dev/null 2>&1
  git push --quiet --set-upstream compiledLibraries upstreamFiles 
}

setup_git
commit_website_files
upload_files