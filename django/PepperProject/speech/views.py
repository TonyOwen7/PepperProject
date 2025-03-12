from django.shortcuts import render, get_object_or_404, redirect
from django.contrib.auth.decorators import login_required
from .models import Speech
from .forms import SpeechForm

@login_required
def speech_list(request):
    speeches = Speech.objects.filter(user=request.user)
    return render(request, 'speech/user_speeches.html', {'speeches': speeches})

@login_required
def create_speech(request):
    if request.method == 'POST':
        form = SpeechForm(request.POST)
        if form.is_valid():
            speech = form.save(commit=False)
            speech.user = request.user
            speech.save()
            return redirect('/speech/speech_list')
    else:
        form = SpeechForm()
    return render(request, 'speech/speech_form.html', {'form': form})

@login_required
def edit_speech(request, speech_id):
    speech = get_object_or_404(Speech, id=speech_id, user=request.user)
    if request.method == 'POST':
        form = SpeechForm(request.POST, instance=speech)
        if form.is_valid():
            form.save()
            return redirect('/speech/speech_list')
    else:
        form = SpeechForm(instance=speech)
    return render(request, 'speech/speech_form.html', {'form': form})

@login_required
def delete_speech(request, speech_id):
    speech = get_object_or_404(Speech, id=speech_id, user=request.user)
    if request.method == 'POST':
        speech.delete()
        return redirect('/speech/speech_list')
    return render(request, 'speech/speech_confirm_delete.html', {'speech': speech})

@login_required
def play_speech(request, speech_id):
    speech = get_object_or_404(Speech, id=speech_id, user=request.user)
    return redirect('control/speech/', {'speech': speech})