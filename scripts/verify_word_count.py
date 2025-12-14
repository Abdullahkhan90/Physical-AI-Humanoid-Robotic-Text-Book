#!/usr/bin/env python3
"""
Script to verify word count across all textbook modules
Checks that the total word count is between 15,000-20,000 words across all modules
"""

import os
import glob
import re


def count_words_in_file(filepath):
    """
    Count words in a markdown file
    """
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Remove markdown formatting and count actual content words
        # Remove headers, code blocks, links, etc.
        # First, remove code blocks
        content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
        content = re.sub(r'`.*?`', '', content)
        
        # Remove markdown headers and markers
        content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)
        content = re.sub(r'\*\*(.*?)\*\*', r'\1', content)  # Bold
        content = re.sub(r'\*(.*?)\*', r'\1', content)    # Italic
        content = re.sub(r'\[(.*?)\]\(.*?\)', r'\1', content)  # Links
        
        # Count words
        words = re.findall(r'\b\w+\b', content)
        return len(words)
    except Exception as e:
        print(f"Error counting words in {filepath}: {e}")
        return 0


def main():
    """
    Main function to count words across all modules
    """
    total_word_count = 0
    file_counts = {}
    
    # Find all markdown files in the docs directory
    module_dirs = [
        "docusaurus/docs/module-1-ros2",
        "docusaurus/docs/module-2-digital-twin",
        "docusaurus/docs/module-3-ai-brain", 
        "docusaurus/docs/module-4-vla"
    ]
    
    for module_dir in module_dirs:
        if os.path.exists(module_dir):
            md_files = glob.glob(os.path.join(module_dir, "*.md"))
            
            for md_file in md_files:
                word_count = count_words_in_file(md_file)
                file_counts[md_file] = word_count
                total_word_count += word_count
                print(f"{md_file}: {word_count} words")
        else:
            print(f"Module directory does not exist: {module_dir}")
    
    print(f"\nTotal word count across all modules: {total_word_count}")
    
    # Check if within required range
    MIN_WORDS = 15000
    MAX_WORDS = 20000
    
    if MIN_WORDS <= total_word_count <= MAX_WORDS:
        print(f"[PASS] Word count is within the required range of {MIN_WORDS:,}-{MAX_WORDS:,} words")
        status = "PASS"
    else:
        print(f"[FAIL] Word count is OUTSIDE the required range of {MIN_WORDS:,}-{MAX_WORDS:,} words")
        print(f"Current word count: {total_word_count:,} - Additional content needed!")
        status = "FAIL"
    
    # Write results to a file for verification
    with open("word_count_verification.txt", "w") as f:
        f.write(f"Total word count: {total_word_count}\n")
        f.write(f"Range: {MIN_WORDS:,} - {MAX_WORDS:,}\n")
        f.write(f"Status: {status}\n")
        f.write("\nBreakdown by file:\n")
        for filepath, count in file_counts.items():
            f.write(f"{filepath}: {count} words\n")
    
    return status == "PASS"


if __name__ == "__main__":
    main()