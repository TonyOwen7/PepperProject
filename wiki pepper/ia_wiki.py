import requests
from bs4 import BeautifulSoup
import re

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
        results_with_scores = []
        keywords = set(re.findall(r'\w+', question.lower()))
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

            page_words = set(re.findall(r'\w+', page_text))
            common_words_count = len(keywords & page_words)
            results_with_scores.append((common_words_count, page_text))

        if results_with_scores:
            best_result = max(results_with_scores, key=lambda x: x[0])
            return best_result[1], best_result[0]  # Return text and score
        else:
            return None, 0  # No relevant content found

    except Exception as e:
        return f"Erreur lors de la récupération des informations sur Wikipédia : {e}", 0

def rechercher_wikihow(question):
    try:
        search_url = f"https://www.wikihow.com/wikiHowTo?search={question}"
        response = requests.get(search_url)
        response.raise_for_status()
        soup = BeautifulSoup(response.text, 'html.parser')

        first_article = soup.find("a", class_="result_link")
        if not first_article:
            return None, 0  # No articles found

        article_url = first_article["href"]
        article_title = first_article.get_text().strip()
        article_response = requests.get(article_url)
        article_response.raise_for_status()
        article_soup = BeautifulSoup(article_response.text, "html.parser")
        summary = article_soup.find("div", class_="mf-section-0")

        if summary:
            article_text = summary.get_text().strip().lower()
            keywords = set(re.findall(r'\w+', question.lower()))
            page_words = set(re.findall(r'\w+', article_text))
            common_words_count = len(keywords & page_words)

            return f"{article_title}: {article_text}", common_words_count
        else:
            return f"{article_title}: Article trouvé, mais aucun résumé n'est disponible.", 1  # Low score if no summary
    except Exception as e:
        return f"Erreur lors de la récupération des informations sur WikiHow : {e}", 0

def rechercher_information(question):
    response_wikipedia, score_wikipedia = rechercher_wikipedia(question)
    response_wikihow, score_wikihow = rechercher_wikihow(question)

    # Choose the response with the highest score
    if score_wikipedia >= score_wikihow:
        if response_wikipedia:
            return f"Wikipedia: {response_wikipedia}"
    if response_wikihow:
        return f"WikiHow: {response_wikihow}"
    return "Je n'ai trouvé aucune information pertinente. Essayez d'autres mots-clés."

def ia_repondre():
    print("Posez-moi une question :")
    while True:
        question = input("Vous: ")
        if question.lower() in ["quit", "exit", "stop"]:
            print("IA: Au revoir!")
            break

        reponse = rechercher_information(question)
        print(f"IA: {reponse}\n")

if __name__ == "__main__":
    ia_repondre()
