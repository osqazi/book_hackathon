#!/usr/bin/env python3
"""
Script to vectorize the Humanoid Robotics Book content and store it in Qdrant.
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Any
import hashlib

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from openai import OpenAI
import numpy as np


class BookVectorizer:
    def __init__(self, api_key: str, endpoint: str, openai_api_key: str = None):
        """
        Initialize the BookVectorizer with Qdrant credentials.

        Args:
            api_key: Qdrant API key
            endpoint: Qdrant cluster endpoint
            openai_api_key: OpenAI API key for embeddings (optional, can use environment variable)
        """
        # Set up Qdrant client
        self.qdrant_client = QdrantClient(
            url=endpoint,
            api_key=api_key,
            timeout=60  # 60 seconds timeout
        )

        # Set up OpenAI client for embeddings
        openai_key = openai_api_key or os.getenv("OPENAI_API_KEY")
        if not openai_key:
            raise ValueError("OpenAI API key is required. Set OPENAI_API_KEY environment variable or pass openai_api_key parameter.")

        self.openai_client = OpenAI(api_key=openai_key)

        # Collection name for the book content
        self.collection_name = "humanoid_robotics_book"

    def extract_content_from_markdown(self, file_path: str) -> List[Dict[str, Any]]:
        """
        Extract content from a markdown file, splitting into meaningful chunks.

        Args:
            file_path: Path to the markdown file

        Returns:
            List of content chunks with metadata
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Remove frontmatter (content between ---)
        content = re.sub(r'---.*?---', '', content, flags=re.DOTALL)

        # Split content into chunks based on headings
        chunks = []

        # Split by H1, H2, H3 headings
        sections = re.split(r'\n(?=#)', content)

        current_section_title = "Introduction"
        current_section_content = ""

        for section in sections:
            if section.strip():
                # Check if this section starts with a heading
                heading_match = re.match(r'#* (.+)', section.strip())

                if heading_match:
                    # Save previous section if it exists
                    if current_section_content.strip():
                        chunks.append({
                            'title': current_section_title.strip(),
                            'content': current_section_content.strip(),
                            'file_path': file_path,
                            'section': current_section_title.strip()
                        })

                    # Start new section
                    current_section_title = heading_match.group(1)
                    current_section_content = section.strip()
                else:
                    current_section_content += section

        # Add the last section
        if current_section_content.strip():
            chunks.append({
                'title': current_section_title.strip(),
                'content': current_section_content.strip(),
                'file_path': file_path,
                'section': current_section_title.strip()
            })

        # Further split large chunks
        final_chunks = []
        for chunk in chunks:
            content = chunk['content']
            if len(content) > 2000:  # If chunk is too large
                # Split by paragraphs
                paragraphs = content.split('\n\n')
                current_chunk = ""

                for para in paragraphs:
                    if len(current_chunk + para) < 2000:
                        current_chunk += "\n\n" + para
                    else:
                        if current_chunk.strip():
                            final_chunks.append({
                                'title': chunk['title'],
                                'content': current_chunk.strip(),
                                'file_path': chunk['file_path'],
                                'section': chunk['section']
                            })
                        current_chunk = para

                # Add remaining content
                if current_chunk.strip():
                    final_chunks.append({
                        'title': chunk['title'],
                        'content': current_chunk.strip(),
                        'file_path': chunk['file_path'],
                        'section': chunk['section']
                    })
            else:
                final_chunks.append(chunk)

        return final_chunks

    def get_embedding(self, text: str) -> List[float]:
        """
        Get embedding for text using OpenAI's embedding API.

        Args:
            text: Text to embed

        Returns:
            Embedding vector as a list of floats
        """
        response = self.openai_client.embeddings.create(
            input=text,
            model="text-embedding-ada-002"  # Using OpenAI's text embedding model
        )
        return response.data[0].embedding

    def create_collection(self):
        """
        Create the Qdrant collection for storing book content.
        """
        # Delete collection if it exists (for re-indexing)
        try:
            self.qdrant_client.delete_collection(self.collection_name)
            print(f"Deleted existing collection: {self.collection_name}")
        except:
            pass  # Collection doesn't exist, which is fine

        # Create new collection with appropriate vector size (1536 for OpenAI embeddings)
        self.qdrant_client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE)
        )
        print(f"Created collection: {self.collection_name}")

    def vectorize_and_store(self, docs_dir: str = "./docs"):
        """
        Vectorize all markdown files in the docs directory and store in Qdrant.

        Args:
            docs_dir: Directory containing markdown files
        """
        # Create the collection
        self.create_collection()

        # Find all markdown files in the docs directory
        md_files = list(Path(docs_dir).rglob("*.md"))
        print(f"Found {len(md_files)} markdown files to process")

        points = []
        point_id = 0

        for file_path in md_files:
            print(f"Processing: {file_path}")

            try:
                # Extract content chunks from the file
                chunks = self.extract_content_from_markdown(str(file_path))

                for chunk in chunks:
                    # Get embedding for the content
                    embedding = self.get_embedding(chunk['content'])

                    # Create a unique ID for this chunk
                    content_hash = hashlib.md5(f"{chunk['file_path']}_{chunk['title']}_{chunk['content'][:100]}".encode()).hexdigest()
                    point_id = int(content_hash[:8], 16)  # Convert first 8 hex chars to int

                    # Create point for Qdrant
                    point = PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={
                            'title': chunk['title'],
                            'content': chunk['content'],
                            'file_path': str(chunk['file_path']),
                            'section': chunk['section'],
                            'source_file': os.path.basename(chunk['file_path'])
                        }
                    )

                    points.append(point)

                    # Batch insert every 100 points or when we have processed all chunks
                    if len(points) >= 100:
                        self.qdrant_client.upsert(
                            collection_name=self.collection_name,
                            points=points
                        )
                        print(f"Upserted {len(points)} points to collection")
                        points = []

            except Exception as e:
                print(f"Error processing {file_path}: {str(e)}")
                continue

        # Insert any remaining points
        if points:
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            print(f"Upserted final {len(points)} points to collection")

        print(f"Successfully vectorized and stored {len(md_files)} files in Qdrant collection: {self.collection_name}")

    def search(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search the vectorized book content.

        Args:
            query: Search query
            limit: Number of results to return

        Returns:
            List of matching chunks with metadata
        """
        query_embedding = self.get_embedding(query)

        hits = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        results = []
        for hit in hits:
            results.append({
                'title': hit.payload.get('title', ''),
                'content': hit.payload.get('content', ''),
                'file_path': hit.payload.get('file_path', ''),
                'section': hit.payload.get('section', ''),
                'score': hit.score
            })

        return results


def main():
    # Get credentials from environment variables or command line
    import sys

    if len(sys.argv) < 3:
        print("Usage: python vectorize_book.py <qdrant_api_key> <qdrant_endpoint> [openai_api_key]")
        sys.exit(1)

    qdrant_api_key = sys.argv[1]
    qdrant_endpoint = sys.argv[2]
    openai_api_key = sys.argv[3] if len(sys.argv) > 3 else None

    # Initialize the vectorizer
    vectorizer = BookVectorizer(
        api_key=qdrant_api_key,
        endpoint=qdrant_endpoint,
        openai_api_key=openai_api_key
    )

    # Vectorize and store the book content
    vectorizer.vectorize_and_store()

    # Test a search to verify everything is working
    print("\nTesting search functionality...")
    test_results = vectorizer.search("What is ROS 2?", limit=3)

    print(f"Search results for 'What is ROS 2?':")
    for i, result in enumerate(test_results, 1):
        print(f"{i}. {result['title']}")
        print(f"   Score: {result['score']:.3f}")
        print(f"   File: {result['file_path']}")
        print(f"   Preview: {result['content'][:200]}...")
        print()


if __name__ == "__main__":
    main()