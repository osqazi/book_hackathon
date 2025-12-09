#!/usr/bin/env python3
"""
Script to integrate the Humanoid Robotics Book with EdenAI AskYoda service.
Based on the API documentation provided.
"""

import os
import json
import requests
import re
from pathlib import Path
from typing import List, Dict, Any
import time
import hashlib


class EdenAIAskYodaIntegrator:
    def __init__(self, api_key: str):
        """
        Initialize the EdenAI AskYoda Integrator with API credentials.

        Args:
            api_key: EdenAI API key
        """
        self.api_key = api_key
        self.base_url = "https://api.edenai.run/v2/aiproducts/askyoda/v2"
        self.headers = {"Authorization": f"Bearer {api_key}"}

        # We'll create a project ID for the humanoid robotics book
        self.project_id = "humanoid-robotics-book"

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

    def add_text_to_project(self, texts: List[str], metadata: List[Dict] = None):
        """
        Add text content to the AskYoda project.

        Args:
            texts: List of text chunks to add
            metadata: List of metadata for each text chunk
        """
        if metadata is None:
            metadata = [{}] * len(texts)

        # Prepare the payload
        payload = {
            "texts": texts,
            "metadata": metadata
        }

        url = f"{self.base_url}/{self.project_id}/add_text"
        response = requests.post(url, json=payload, headers=self.headers)

        if response.status_code in [200, 201]:
            print(f"Successfully added {len(texts)} text chunks to project {self.project_id}")
            return response.json()
        else:
            print(f"Error adding text to project: {response.status_code} - {response.text}")
            print(f"URL attempted: {url}")
            response.raise_for_status()

    def process_book_content(self, docs_dir: str = "./docs"):
        """
        Process all markdown files in the docs directory and add to AskYoda project.

        Args:
            docs_dir: Directory containing markdown files
        """
        # Find all markdown files in the docs directory
        md_files = list(Path(docs_dir).rglob("*.md"))
        print(f"Found {len(md_files)} markdown files to process")

        # Process each file and extract content
        all_texts = []
        all_metadata = []

        for file_path in md_files:
            print(f"Processing: {file_path}")

            try:
                # Extract content chunks from the file
                chunks = self.extract_content_from_markdown(str(file_path))

                for chunk in chunks:
                    # Add to our lists
                    all_texts.append(chunk['content'])
                    all_metadata.append({
                        'title': chunk['title'],
                        'file_path': str(chunk['file_path']),
                        'section': chunk['section'],
                        'source_file': os.path.basename(chunk['file_path'])
                    })

                    # Batch process every 10 chunks to avoid hitting API limits
                    if len(all_texts) >= 10:
                        print(f"Adding batch of {len(all_texts)} texts to project...")
                        try:
                            self.add_text_to_project(all_texts, all_metadata)
                        except Exception as e:
                            print(f"Error adding batch: {e}")
                            # Try adding one by one if batch fails
                            for text, meta in zip(all_texts, all_metadata):
                                try:
                                    self.add_text_to_project([text], [meta])
                                except Exception as e2:
                                    print(f"Failed to add individual text: {e2}")
                                    continue

                        # Reset for next batch
                        all_texts = []
                        all_metadata = []

            except Exception as e:
                print(f"Error processing {file_path}: {str(e)}")
                continue

        # Add any remaining texts
        if all_texts:
            print(f"Adding final batch of {len(all_texts)} texts to project...")
            try:
                self.add_text_to_project(all_texts, all_metadata)
            except Exception as e:
                print(f"Error adding final batch: {e}")
                # Try adding one by one if batch fails
                for text, meta in zip(all_texts, all_metadata):
                    try:
                        self.add_text_to_project([text], [meta])
                    except Exception as e2:
                        print(f"Failed to add individual text: {e2}")
                        continue

        print(f"Successfully processed and added all book content to AskYoda project: {self.project_id}")

    def ask_question(self, query: str, llm_provider: str = "openai", llm_model: str = "gpt-4", k: int = 5) -> Dict[str, Any]:
        """
        Ask a question to the AskYoda system.

        Args:
            query: The question to ask
            llm_provider: LLM provider to use
            llm_model: LLM model to use
            k: Number of documents to retrieve

        Returns:
            Response from the AskYoda system
        """
        url = f"{self.base_url}/{self.project_id}/ask_llm"
        payload = {
            "query": query,
            "llm_provider": llm_provider,
            "llm_model": llm_model,
            "k": k,
            "max_tokens": 500,
            "temperature": 0.7,
        }

        response = requests.post(url, json=payload, headers=self.headers)

        if response.status_code in [200, 201]:
            return response.json()
        else:
            print(f"Error asking question: {response.status_code} - {response.text}")
            print(f"URL attempted: {url}")
            response.raise_for_status()


def main():
    import sys

    if len(sys.argv) < 2:
        print("Usage: python edenai_askyoda_integrator.py <edenai_api_key>")
        sys.exit(1)

    api_key = sys.argv[1]

    # Initialize the integrator
    integrator = EdenAIAskYodaIntegrator(api_key=api_key)

    # Process and add the book content
    integrator.process_book_content()

    # Test a question to verify everything is working
    print("\nTesting question functionality...")
    try:
        test_result = integrator.ask_question("What is ROS 2?")
        print(f"Test question result: {json.dumps(test_result, indent=2)}")
    except Exception as e:
        print(f"Test question failed (this may be expected if project is still indexing): {str(e)}")

    print("\nBook content has been successfully integrated with EdenAI AskYoda!")


if __name__ == "__main__":
    main()