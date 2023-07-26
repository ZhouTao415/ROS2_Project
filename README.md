# ROS2_Project

## - VMware虚拟机的Ubuntu里 git 的使用
1. 安装git
```bash
sudo apt-get install git
```
2.  将项目从github上clone到本地
```bash
git clone https://github.com/ZhouTao415/ROS2_Project.git
```
3. 设置本地git的用名和邮
```bash
git config --global user.name "YOUR NAME"
git config --global user.name "YOUR EMAIL ADDRESS"
```
4. 检查是否已经有SSH密钥
```bash
ls ~/.ssh
```
5. 生成新的SSH密钥：如果第4步中没有发现SSH密钥
```bash
ssh-keygen -t rsa -C "yourmail@example.com"
```
6. 将您的 SSH 账户添加到 GitHub：
复制SSH的内容,并登录到您的GitHub帐户，转到“设置”（设置）>“SSH和GPG密钥”（SSH和GPG钥匙）>“新SSH密钥”（新SSH钥匙）
```bash
cat ~/.ssh/id_rsa.pub
```
7. 更改修改远程仓库的URL：
现在，您需要将远程仓库的URL从HTTPS更改为SSH。在您的ROS2_Project目录下，使用以下命令远程仓库的URL：
```bash
git remote set-url origin git@github.com:ZhouTao415/ROS2_Project.git
```
7. 自动化更改：现在，您应该能够通过 SSH 进行认证。再次尝试执行 git 自动化命令：
```bash
git push
```


