# Setup our quail submodule
Write-Host "Setting up quail submodule" -ForegroundColor Blue
git submodule update --init --recursive
Write-Host "Quail submodule setup complete" -ForegroundColor Green

# Symlink the quail submodule
Write-Host "Symlinking quail submodule" -ForegroundColor Blue


# Make sure we can build
Write-Host "Trying to build robot code" -ForegroundColor Blue
.\gradlew clean build
