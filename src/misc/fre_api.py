import requests
import json

base_url = "http://192.168.1.4:8000/fre2024"    # Use for competition
api_key = "U0jvVpacI5rXiSnWVwI9Z"    # Insert api key

headers = {
    "Content-Type": "application/json",
    "x-api-key": api_key
}

def get_positions_from_api(url='http://localhost:8000/fre2024/task4/get-positions'):
    headers = {
        'accept': 'application/json'
    }

    response = requests.get(url, headers=headers)

    if response.status_code == 200:
        # Parse the JSON response
        data = response.json()
        return data
    else:
        print(f"Failed to fetch data. Status code: {response.status_code}")
        print(f"Response content: {response.text}")
        return None