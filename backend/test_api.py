import requests
import json

# Test the backend API endpoints
BASE_URL = "https://hafizabdullah9-backend-rag-chatbot.hf.space"

def test_endpoints():
    print("Testing backend API endpoints...")
    
    # Test health endpoint
    try:
        response = requests.get(f"{BASE_URL}/health")
        print(f"Health endpoint: {response.status_code} - {response.text}")
    except Exception as e:
        print(f"Health endpoint error: {e}")
    
    # Test root endpoint
    try:
        response = requests.get(f"{BASE_URL}/")
        print(f"Root endpoint: {response.status_code} - {response.text}")
    except Exception as e:
        print(f"Root endpoint error: {e}")
    
    # Test API endpoints
    try:
        headers = {"Content-Type": "application/json"}
        payload = {"question": "test"}
        response = requests.post(f"{BASE_URL}/api/query", headers=headers, data=json.dumps(payload))
        print(f"API query endpoint: {response.status_code} - {response.text}")
    except Exception as e:
        print(f"API query endpoint error: {e}")
    
    # Test fallback endpoints
    try:
        headers = {"Content-Type": "application/json"}
        payload = {"question": "test"}
        response = requests.post(f"{BASE_URL}/query", headers=headers, data=json.dumps(payload))
        print(f"Fallback query endpoint: {response.status_code} - {response.text}")
    except Exception as e:
        print(f"Fallback query endpoint error: {e}")

if __name__ == "__main__":
    test_endpoints()