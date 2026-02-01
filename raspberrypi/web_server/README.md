## Web Server Setup

These instructions assume you have the contents of the `raspberrypi` folder copied to your Raspberry Pi and you're in that folder.

1. Update the system and install packages

   ```bash
   sudo apt update
   sudo apt upgrade -y
   sudo apt install -y nginx python3 python3-picamera2 python3-opencv python3-numpy python3-flask python3-werkzeug python3-serial
   ```

2. Give yourself permission to access the serial port
   ```bash
   sudo usermod -a -G dialout $USER
   reboot
   ```

3. Deploy nginx config and site files

   Move the default nginx site config (it may cause problems):

   ```bash
   sudo mv /etc/nginx/sites-enabled/default /etc/nginx/sites-available/default
   ```

   ```bash
   cd web_server

   sudo ln -s "$(pwd)/nginx.conf" /etc/nginx/conf.d/lnx-infrabot.conf
   sudo cp -r site/* /var/www/html/

   sudo chown -R www-data:www-data /var/www/html
   sudo chmod -R 755 /var/www/html
   ```

   ```bash
   sudo systemctl restart nginx
   ```

4. Install and enable the service

   ```bash
   cd ..

   sudo ln -s "$(pwd)/robot.service" /etc/systemd/system/robot.service
   sudo systemctl daemon-reload
   sudo systemctl enable robot
   ```
