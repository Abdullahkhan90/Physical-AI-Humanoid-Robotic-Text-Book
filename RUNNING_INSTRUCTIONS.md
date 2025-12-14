# Running the AI Native Textbook Project

## Project Overview
This is a Docusaurus-based project for an AI Native Textbook on Physical AI & Humanoid Robotics.

## Project Structure
- Root directory: Contains the main package.json
- `/docusaurus` directory: Contains the actual Docusaurus website

## How to Run the Project

1. **Navigate to the docusaurus directory:**
   ```bash
   cd docusaurus
   ```

2. **Install dependencies:**
   ```bash
   npm install
   ```

3. **Start the development server:**
   ```bash
   npm run start
   ```

## Alternative Method

From the root directory, you can use the scripts defined in the root package.json:
```bash
npm install  # Install root dependencies
npm run dev  # Start the docusaurus development server
```

## Troubleshooting

If you encounter npm installation issues on Windows:

1. **Clear npm cache:**
   ```bash
   npm cache clean --force
   ```

2. **Delete node_modules and package-lock.json in the docusaurus directory:**
   ```bash
   rm -rf docusaurus/node_modules
   rm docusaurus/package-lock.json
   ```

3. **Try installing with legacy peer deps:**
   ```bash
   cd docusaurus
   npm install --legacy-peer-deps
   ```

4. **Or try installing with --force flag:**
   ```bash
   cd docusaurus
   npm install --force
   ```

5. **If you get network errors (ECONNRESET):**
   - Check your internet connection
   - Try using a different network or VPN if behind a corporate firewall
   - Configure npm proxy settings if needed:
     ```bash
     npm config set proxy http://your-proxy:port
     npm config set https-proxy http://your-proxy:port
     ```

6. **If you get permission errors (EPERM):**
   - Run the command prompt as Administrator
   - Or try:
     ```bash
     npm install --no-audit --no-fund
     ```

## Expected Result

Once successfully running, you should see the Docusaurus development server start and the site will be accessible at http://localhost:3000

The site will contain modules covering:
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac)
- Module 4: Vision-Language-Action (VLA)