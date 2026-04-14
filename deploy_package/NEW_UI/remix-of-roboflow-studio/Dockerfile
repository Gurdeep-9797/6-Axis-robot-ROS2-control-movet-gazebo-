# Frontend Dockerfile
# RoboForge v8.0 Online IDE

FROM node:20-slim

WORKDIR /app

# Install dependencies
COPY package.json package-lock.json* ./
RUN npm install

# Copy source code
COPY . .

# Expose Vite port (default 5173, but we'll map to 3000 in compose)
EXPOSE 5173

# Run development server with host access
CMD ["npm", "run", "dev", "--", "--host", "0.0.0.0"]
