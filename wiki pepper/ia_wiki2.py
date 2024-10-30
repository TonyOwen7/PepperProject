from flask import Flask, request, jsonify
from flask_cors import CORS
import requests
from bs4 import BeautifulSoup
import re

app = Flask(__name__)
CORS(app)  # Enable cross-origin requests for front-end access

# Utility Functions
def extract_keywords(text):
    """Extract keywords from text, ignoring common words and special characters."""
    return set(re.findall(r'\w+', text.lower()))

def find_most_relevant_paragraphs(text, question_keywords, max_paragraphs=3):
    """Finds the most relevant paragraphs based on keyword overlap."""
    paragraphs = [p.strip() for p in text.split('\n') if p.strip()]
    scored_paragraphs = []

    for paragraph in paragraphs:
        paragraph_keywords = extract_keywords(paragraph)
        overlap = len(question_keywords & paragraph_keywords)
        
        if overlap > 0:  # Consider only paragraphs with at least one keyword match
            scored_paragraphs.append((overlap, paragraph))

    # Sort paragraphs by relevance and return the most relevant ones
    scored_paragraphs.sort(reverse=True, key=lambda x: x[0])
    top_paragraphs = [p[1] for p in scored_paragraphs[:max_paragraphs]]
    return " ".join(top_paragraphs)

def get_text_content(soup):
    """Extract relevant text content, ignoring images, videos, and non-text elements."""
    for tag in soup(['img', 'video', 'script', 'style', 'aside', 'footer', 'noscript']):
        tag.decompose()
    return soup.get_text(separator="\n", strip=True)

# Wikipedia and WikiHow Functions
def rechercher_wikipedia(question, langue="fr"):
    url = f"https://{langue}.wikipedia.org/w/api.php"
    params = {
        "action": "query",
        "list": "search",
        "srsearch": question,
        "format": "json",
    }
    try:
        response = requests.get(url, params=params)
        response.raise_for_status()
        data = response.json()
        keywords = extract_keywords(question)
        blacklist = {"étudiant", "fête", "bière", "événement"}

        for result in data["query"]["search"]:
            page_id = result["pageid"]
            page_title = result["title"].lower()
            if any(word in page_title for word in blacklist):
                continue

            content_url = f"https://{langue}.wikipedia.org/w/api.php?action=query&prop=extracts&pageids={page_id}&exintro=&format=json"
            content_response = requests.get(content_url)
            content_response.raise_for_status()
            content_data = content_response.json()
            page_content = content_data["query"]["pages"][str(page_id)]["extract"]
            page_text = BeautifulSoup(page_content, "html.parser").get_text().strip().lower()

            if any(word in page_text for word in blacklist):
                continue

            relevant_text = find_most_relevant_paragraphs(page_text, keywords)
            return relevant_text

        return None  # No relevant content found

    except Exception as e:
        return f"Erreur lors de la récupération des informations sur Wikipédia : {e}"

def rechercher_wikihow(question):
    try:
        search_url = f"https://fr.wikihow.com/wikiHowTo?search={question}"
        response = requests.get(search_url)
        response.raise_for_status()
        soup = BeautifulSoup(response.text, 'html.parser')

        articles = soup.find_all("a", class_="result_link")
        if not articles:
            direct_url = f"https://fr.wikihow.com/{question.capitalize()}"
            article_response = requests.get(direct_url)
            if article_response.status_code == 200:
                article_soup = BeautifulSoup(article_response.text, "html.parser")
                main_content = get_text_content(article_soup)
                
                if main_content:
                    keywords = extract_keywords(question)
                    return find_most_relevant_paragraphs(main_content, keywords)

            return "Aucun article pertinent trouvé sur WikiHow."

        keywords = extract_keywords(question)
        for article in articles:
            article_url = article["href"]
            article_response = requests.get(article_url)
            article_response.raise_for_status()
            article_soup = BeautifulSoup(article_response.text, "html.parser")
            
            main_content = ""
            for section in article_soup.find_all("div", class_="step"):
                main_content += section.get_text().strip() + "\n"
            
            if main_content:
                return find_most_relevant_paragraphs(main_content, keywords)

        return "Aucun article pertinent trouvé sur WikiHow."

    except Exception as e:
        return f"Erreur lors de la récupération des informations sur WikiHow : {e}"

def rechercher_information(question):
    response_wikipedia = rechercher_wikipedia(question)
    response_wikihow = rechercher_wikihow(question)

    if response_wikipedia and response_wikihow:
        return f"Wikipedia: {response_wikipedia}" if len(response_wikipedia) >= len(response_wikihow) else f"WikiHow: {response_wikihow}"
    elif response_wikipedia:
        return f"Wikipedia: {response_wikipedia}"
    elif response_wikihow:
        return f"WikiHow: {response_wikihow}"
    else:
        return "Je n'ai trouvé aucune information pertinente. Essayez d'autres mots-clés."

# Flask API Route
@app.route('/ask', methods=['POST'])
def ask():
    data = request.json
    question = data.get("question")
    if not question:
        return jsonify({"error": "No question provided"}), 400

    # Use the information retrieval function
    answer = rechercher_information(question)
    return jsonify({"answer": answer})

if __name__ == "__main__":
    app.run(debug=True)
