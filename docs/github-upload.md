# 第一次上传到GitHub

推荐使用 **GitHub Desktop**，比命令行更适合第一次使用。

## 方法一：GitHub Desktop

1. 注册并登录GitHub账号。
2. 下载并安装GitHub Desktop。
3. 打开GitHub Desktop，选择 **File -> Add local repository**。
4. 选择桌面上的《锁相环》文件夹。
5. 如果提示该目录还不是Git仓库，点击 **create a repository**。
6. Name建议填写 `stm32f407-lissajous-pll`。
7. Description可填写：
   `STM32F407 + AD9833 Lissajous figure controller with dual-ADC sampling and digital PLL/FLL.`
8. 不要再次添加README、Git ignore或License，因为文件夹中已经准备好了。
9. 点击 **Create repository**，然后点击 **Publish repository**。
10. 如果希望别人能看到，取消勾选 **Keep this code private**；如果暂时只想备份，就保留私有。

以后修改文件后，只需打开GitHub Desktop：

1. 在左下角Summary填写本次修改内容。
2. 点击 **Commit to main**。
3. 点击顶部 **Push origin**。

## 方法二：直接用网页

1. 登录GitHub，右上角点击 **+ -> New repository**。
2. 仓库名填写 `stm32f407-lissajous-pll`。
3. 不要勾选自动创建README、.gitignore或License。
4. 创建仓库后点击 **uploading an existing file**。
5. 打开桌面《锁相环》文件夹，选中里面的全部文件和子文件夹拖入网页。
6. 提交说明填写 `Initial release: Questions 1-4`。
7. 点击 **Commit changes**。

网页方式适合第一次上传；长期维护建议改用GitHub Desktop。

## 上传前检查

- 不上传手机号、学号、串口截图中的个人信息。
- 不上传Keil的Objects/Listings、编译日志、备份文件。
- 不上传第五问摄像头代码。
- 不上传大体积STM32 HAL/CMSIS副本。
- 未确认第三方驱动版权前，不要随意添加MIT License。
