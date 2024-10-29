import rospy
from std_msgs.msg import String
import requests
from bs4 import BeautifulSoup
import re

def rechercher_wikipedia(question, langue="fr"):
    """Utiliser l'API Wikipedia pour rechercher la question en français."""
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

        # Diviser la question en mots-clés pour le calcul de la pertinence
        keywords = set(re.findall(r'\w+', question.lower()))

        # Mots-clés qui pourraient indiquer un hors sujet
        blacklist = {"étudiant", "fête", "bière", "événement"}

        for result in data["query"]["search"]:
            page_id = result["pageid"]
            page_title = result["title"].lower()

            # Filtrer les résultats avec des titres hors sujet
            if any(word in page_title for word in blacklist):
                continue

            # Récupérer le contenu de la page
            content_url = f"https://{langue}.wikipedia.org/w/api.php?action=query&prop=extracts&pageids={page_id}&exintro=&format=json"
            content_response = requests.get(content_url)
            content_response.raise_for_status()

            content_data = content_response.json()
            page_content = content_data["query"]["pages"][str(page_id)]["extract"]
            page_text = BeautifulSoup(page_content, "html.parser").get_text().strip().lower()

            # Vérifier si le contenu contient des mots clés indésirables
            if any(word in page_text for word in blacklist):
                continue

            # Calculer le score de pertinence
            page_words = set(re.findall(r'\w+', page_text))
            common_words_count = len(keywords & page_words)

            # Ajouter le résultat et son score à la liste
            results_with_scores.append((common_words_count, page_text))

        # Trier les résultats par score de pertinence
        results_with_scores.sort(reverse=True, key=lambda x: x[0])

        # Retourner le meilleur résultat pertinent
        if results_with_scores and results_with_scores[0][0] > 0:
            return results_with_scores[0][1]
        else:
            return "Je n'ai trouvé aucune information pertinente sur cette question. Essayez d'autres mots-clés."

    except Exception as e:
        return f"Erreur lors de la récupération des informations : {e}"

def callback(data):
    """Callback pour traiter les messages audio reçus."""
    question = data.data
    rospy.loginfo("Audio reçu : %s", question)
    
    # Rechercher une réponse à la question reçue
    reponse = rechercher_wikipedia(question, langue="fr")
    rospy.loginfo("IA: %s", reponse)  # Affiche la réponse dans le log ROS
    # Ici, vous pourriez également faire parler le robot Pepper, par exemple:
    # pepper_say(reponse)

def listener():
    """Initialiser le listener ROS pour recevoir des messages audio."""
    rospy.init_node('audio_listener', anonymous=True)
    rospy.Subscriber("/pepper_robot/audio", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
