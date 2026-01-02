"""
Script to crawl the AI Engineering book site and extract all book content.
"""
import requests
from bs4 import BeautifulSoup
import time
import urllib.parse
from typing import List, Tuple

def get_page_content(url: str) -> str:
    """
    Fetch and extract clean text content from a single page.
    """
    try:
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }
        response = requests.get(url, headers=headers)
        response.raise_for_status()
        
        soup = BeautifulSoup(response.content, 'html.parser')
        
        # Remove navigation, headers, footers, and other UI elements
        for element in soup(['nav', 'header', 'footer', 'aside', 'script', 'style']):
            element.decompose()
        
        # Try to find the main content area - common selectors for Docusaurus sites
        main_content = None
        selectors_to_try = [
            'main',  # Most common
            'article',  # Common for content
            '.main-wrapper',  # Docusaurus specific
            '.container',  # General container
            '.theme-doc-markdown',  # Docusaurus markdown content
            '.markdown',  # Markdown content
            '.doc-content',  # Documentation content
            '.content',  # General content area
        ]
        
        for selector in selectors_to_try:
            main_content = soup.select_one(selector)
            if main_content:
                break
        
        # If no specific content area found, use the body
        if not main_content:
            main_content = soup.find('body')
        
        if main_content:
            # Extract text and clean it up
            text = main_content.get_text(separator=' ', strip=True)
            # Clean up multiple spaces and newlines
            import re
            text = re.sub(r'\s+', ' ', text)
            return text
        else:
            return ""
            
    except Exception as e:
        print(f"Error fetching {url}: {e}")
        return ""

def crawl_book_site(base_url: str = "https://ai-engineering-book.vercel.app") -> List[Tuple[str, str]]:
    """
    Crawl all book pages and extract their content.
    """
    book_urls = [
        "/book/introduction",
        "/book/module1-introduction",
        "/book/module1-chapter1-middleware",
        "/book/module1-chapter2-ros2-core-concepts",
        "/book/module1-chapter3-rclpy-bridge",
        "/book/module1-chapter4-urdf-humanoids",
        "/book/module2-introduction",
        "/book/module2-chapter1-physics-simulation",
        "/book/module2-chapter2-gazebo-physics",
        "/book/module2-chapter3-unity-rendering",
        "/book/module2-chapter4-simulating-sensors",
        "/book/module3-introduction",
        "/book/module3-chapter1-advanced-perception-and-training",
        "/book/module3-chapter2-nvidia-isaac-sim",
        "/book/module3-chapter3-isaac-ros",
        "/book/module3-chapter4-nav2-path-planning",
        "/book/module4-introduction",
        "/book/module4-chapter1-llms-and-robotics",
        "/book/module4-chapter2-voice-to-action",
        "/book/module4-chapter3-cognitive-planning",
        "/book/module4-chapter4-capstone-project"
    ]
    
    all_content = []
    
    for url_path in book_urls:
        full_url = urllib.parse.urljoin(base_url, url_path)
        print(f"Fetching content from: {full_url}")
        
        content = get_page_content(full_url)
        
        if content:
            all_content.append((full_url, content))
            print(f"  Retrieved {len(content)} characters")
        else:
            print(f"  No content retrieved")
        
        # Be respectful to the server
        time.sleep(1)
    
    return all_content

def save_combined_content(content_list: List[Tuple[str, str]], output_file: str = "crawled_book_content.txt"):
    """
    Combine all content and save to a file.
    """
    with open(output_file, 'w', encoding='utf-8') as f:
        for url, content in content_list:
            f.write(f"URL: {url}\n")
            f.write("="*80 + "\n")
            f.write(content + "\n")
            f.write("\n" + "="*80 + "\n\n")
    
    print(f"Combined content saved to {output_file}")

if __name__ == "__main__":
    print("Starting to crawl the AI Engineering book site...")
    book_content = crawl_book_site()
    
    print(f"\nCrawled {len(book_content)} pages successfully.")
    
    if book_content:
        save_combined_content(book_content)
        
        # Print summary
        total_chars = sum(len(content) for _, content in book_content)
        print(f"Total characters extracted: {total_chars}")
        print("Content extraction completed!")
    else:
        print("No content was extracted.")