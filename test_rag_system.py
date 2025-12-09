#!/usr/bin/env python3
"""
Test script to verify the RAG system is working correctly.
"""

import asyncio
import requests
import json
import os
from pathlib import Path

def test_backend():
    """Test if the backend is running and accessible."""
    try:
        response = requests.get("http://localhost:8001/health", timeout=10)
        if response.status_code == 200:
            print("Backend is running and accessible")
            return True
        else:
            print(f"Backend returned status code: {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print("Backend is not running. Please start it with: uvicorn backend.main:app --reload")
        return False
    except Exception as e:
        print(f"Error connecting to backend: {e}")
        return False

def test_search():
    """Test the search functionality."""
    try:
        response = requests.get(
            "http://localhost:8001/search",
            params={"query": "ROS 2", "limit": 3},
            timeout=30
        )
        if response.status_code == 200:
            data = response.json()
            print(f"Search is working. Found {len(data.get('results', []))} results for 'ROS 2'")
            return True
        else:
            print(f"Search failed with status code: {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except Exception as e:
        print(f"Error during search test: {e}")
        return False

def test_chat():
    """Test the chat functionality."""
    try:
        payload = {
            "query": "What is ROS 2?",
            "max_tokens": 200,
            "temperature": 0.7
        }
        response = requests.post(
            "http://localhost:8001/chat",
            json=payload,
            timeout=60
        )
        if response.status_code == 200:
            data = response.json()
            print("Chat is working. Response received.")
            print(f"Query: {data.get('query', '')[:50]}...")
            print(f"Response length: {len(data.get('response', ''))} characters")
            return True
        else:
            print(f"Chat failed with status code: {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except Exception as e:
        print(f"Error during chat test: {e}")
        return False

def main():
    print("Testing RAG System...")
    print("=" * 50)

    # Test backend health
    if not test_backend():
        print("\nPlease start the backend server with this command:")
        print("cd backend && python -c \"from main import app; import uvicorn; uvicorn.run(app, host='0.0.0.0', port=8001)\"")
        return

    # Test search functionality
    print("\nTesting search functionality...")
    search_ok = test_search()

    # Test chat functionality
    print("\nTesting chat functionality...")
    chat_ok = test_chat()

    print("\n" + "=" * 50)
    print("Test Summary:")
    print(f"Backend Health: {'PASS' if True else 'FAIL'}")  # Backend test passed if we got here
    print(f"Search Function: {'PASS' if search_ok else 'FAIL'}")
    print(f"Chat Function: {'PASS' if chat_ok else 'FAIL'}")

    if search_ok and chat_ok:
        print("\nAll tests passed! The RAG system is working correctly.")
        print("\nTo integrate with your Docusaurus site:")
        print("1. Make sure the backend is running on http://localhost:8001")
        print("2. The chat widget is already integrated into all pages via the Layout override")
        print("3. Users can select text on any page and ask questions about it")
        print("4. The system will provide answers based on the book content with source citations")
    else:
        print("\nSome tests failed. Please check the backend implementation.")

if __name__ == "__main__":
    main()