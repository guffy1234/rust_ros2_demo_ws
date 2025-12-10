$instanceName = "Ubuntu2404_Ros2Rust"

$wslPath = "C:\WSL2-Distros\distros"
$installPath = "$wslPath\$instanceName"

New-Item -ItemType Directory -Force -Path $installPath | Out-Null

wsl --unregister $instanceName

wsl --install --web-download --distribution Ubuntu-24.04 --name $instanceName --location $instanceName --no-launch

# Pause to allow instance creation to complete
Write-Host "Waiting 10 seconds for instance creation..."
Start-Sleep -Seconds 10

# Verify the instance was created
Write-Host "Verifying WSL instance $instanceName..."
$wslList = wsl --list --quiet
if ($null -eq $wslList) {
    Write-Host "Failed to list WSL instances. Ensure WSL is installed and functional."
    exit 1
}

# Start the WSL instance and configure it
Write-Host "Configuring WSL instance $instanceName..."

Write-Host "Stopping instance..."
wsl --terminate $instanceName 2>$null

Write-Host "Creating simuser (no password)..."
wsl -d $instanceName -u root -- adduser --disabled-password --no-create-home simuser
wsl -d $instanceName -u root -- mkhomedir_helper simuser   # creates /home/simuser cleanly
wsl -d $instanceName -u root -- passwd -d simuser
wsl -d $instanceName -u root -- usermod -aG adm,dialout,cdrom,sudo,audio,dip,video,plugdev,netdev simuser 
wsl -d $instanceName -u root -- usermod -aG lxd simuser 


Write-Host "Enabling passwordless sudo..."
wsl -d $instanceName -u root -- bash -c "echo 'simuser ALL=(ALL) NOPASSWD: ALL' > /etc/sudoers.d/simuser && chmod 0440 /etc/sudoers.d/simuser"


Write-Host "Setting simuser as default + enabling systemd..."
wsl -d $instanceName -u root -- /bin/bash -c 'cat > /etc/wsl.conf <<EOF
[user]
default = simuser

[boot]
systemd = true
EOF
'

# Write-Host "Creating simuser (100% silent, no prompts) ..."
# wsl -d $instanceName -u root -- /bin/bash -c "
# adduser --disabled-password --no-create-home --gecos ,,,, simuser
# mkhomedir_helper simuser
# passwd -d simuser
# usermod -aG adm,dialout,cdrom,sudo,audio,dip,video,plugdev,netdev,lxd simuser 2>/dev/null || true
# echo 'simuser ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/simuser
# chmod 0440 /etc/sudoers.d/simuser
# cat > /etc/wsl.conf <<EOF
# [user]
# default = simuser

# [boot]
# systemd = true
# EOF
# "

# Terminate the WSL instance to apply changes
Write-Host "Terminating WSL instance to apply changes..."
wsl --terminate $instanceName

# Restart the WSL instance as simuser
Write-Host "Restarting WSL instance as simuser..."
wsl -d $instanceName -u simuser -- whoami
$whoami = wsl -d $instanceName -u simuser -- whoami
if ($whoami -eq "simuser") {
    Write-Host "Successfully logged in as simuser."
} else {
    Write-Host "Failed to log in as simuser. Current user: $whoami"
    exit 1
}


