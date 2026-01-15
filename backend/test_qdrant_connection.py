#!/usr/bin/env python3
"""
Test script to verify Qdrant connection with cloud credentials
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from src.services.qdrant_service import QdrantService

def test_qdrant_connection():
    print("Initializing QdrantService with cloud credentials...")
    try:
        q = QdrantService()
        print("✓ Qdrant service initialized successfully")
        
        if q.client:
            print("✓ Qdrant client is connected")
            
            # Test if we can access the collection
            try:
                collection_info = q.client.get_collection(q.collection_name)
                print(f"✓ Collection {q.collection_name} exists and is accessible")
                print(f"  Collection vectors count: {collection_info.points_count}")
            except Exception as e:
                print(f"✗ Error accessing collection: {e}")
                
            # Test a simple search with an empty query
            try:
                results = q.search([], top_k=1)  # Empty vector for test
                print("✓ Search functionality is working")
            except Exception as e:
                print(f"✗ Error with search functionality: {e}")
        else:
            print("✗ Qdrant client is not initialized properly")
            
    except Exception as e:
        print(f"✗ Error initializing QdrantService: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_qdrant_connection()