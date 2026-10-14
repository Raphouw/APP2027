"""Onglet « Robot » : connexion SSH et lancement du bringup depuis l'interface."""
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from . import robot_ssh as ssh

STATE_COLORS = {ssh.CONNECTED: '#14704b', ssh.BRINGUP: '#14704b', ssh.ERROR: '#b52d3a',
                ssh.CONNECTING: '#846332', ssh.DISCONNECTED: '#5a6b7b'}


class ManagerRobotMixin:
    def build_robot_tab(self):
        self.robot = ssh.RobotLink()
        settings = ssh.load_settings()
        tab = ttk.Frame(self.book, padding=12)
        self.book.add(tab, text='  Robot (SSH)  ')
        self.robot_vars = dict(host=tk.StringVar(value=settings['host']), user=tk.StringVar(value=settings['user']),
                               port=tk.StringVar(value=str(settings['port'])), key_file=tk.StringVar(value=settings['key_file']),
                               ros_domain_id=tk.StringVar(value=str(settings['ros_domain_id'])),
                               model=tk.StringVar(value=settings['model']), lds_model=tk.StringVar(value=settings['lds_model']),
                               remember_password=tk.BooleanVar(value=bool(settings.get('remember_password'))),
                               password=tk.StringVar())
        self._robot_extra = {k: settings[k] for k in ('ros_distro', 'bringup_command', 'camera_port')}
        form = ttk.Frame(tab)
        form.pack(fill='x')
        rows = [('Adresse du robot', 'host', 'ex. 192.168.1.50 ou turtlebot.local'), ('Utilisateur', 'user', ''),
                ('Mot de passe', 'password', 'laissez vide pour utiliser une clé SSH ou le mot de passe mémorisé')]
        for row, (label, key, hint) in enumerate(rows):
            ttk.Label(form, text=label).grid(row=row, column=0, sticky='w', pady=3, padx=(0, 10))
            entry = ttk.Entry(form, textvariable=self.robot_vars[key], width=34, show='•' if key == 'password' else '')
            entry.grid(row=row, column=1, sticky='w')
            ttk.Label(form, text=hint, style='Subtitle.TLabel').grid(row=row, column=2, sticky='w', padx=10)
        remember = ttk.Frame(form)
        remember.grid(row=3, column=1, columnspan=2, sticky='w')
        self.robot_remember_check = ttk.Checkbutton(remember, text='Mémoriser dans le trousseau du système',
                                                    variable=self.robot_vars['remember_password'],
                                                    command=self._robot_remember_toggled)
        self.robot_remember_check.pack(side='left')
        self.robot_forget = ttk.Button(remember, text='Oublier le mot de passe', command=self.robot_forget_clicked)
        self.robot_forget.pack(side='left', padx=10)
        self.robot_password_note = ttk.Label(form, text='', style='Subtitle.TLabel')
        self.robot_password_note.grid(row=4, column=1, columnspan=2, sticky='w')
        ttk.Label(form, text='Clé SSH (option)').grid(row=5, column=0, sticky='w', pady=3)
        ttk.Entry(form, textvariable=self.robot_vars['key_file'], width=34).grid(row=5, column=1, sticky='w')
        ttk.Button(form, text='Parcourir…', command=self._robot_pick_key).grid(row=5, column=2, sticky='w', padx=10)
        ttk.Label(form, text='ROS_DOMAIN_ID').grid(row=6, column=0, sticky='w', pady=3)
        ttk.Entry(form, textvariable=self.robot_vars['ros_domain_id'], width=8).grid(row=6, column=1, sticky='w')
        ttk.Label(form, text='Modèle').grid(row=7, column=0, sticky='w', pady=3)
        ttk.Combobox(form, textvariable=self.robot_vars['model'], values=ssh.MODELS, width=12,
                     state='readonly').grid(row=7, column=1, sticky='w')
        ttk.Label(form, text='vide = celui défini sur le robot (~/.bashrc)', style='Subtitle.TLabel').grid(
            row=7, column=2, sticky='w', padx=10)
        ttk.Label(form, text='Modèle LiDAR').grid(row=8, column=0, sticky='w', pady=3)
        ttk.Combobox(form, textvariable=self.robot_vars['lds_model'], values=ssh.LDS_MODELS, width=12,
                     state='readonly').grid(row=8, column=1, sticky='w')
        ttk.Label(form, text='LDS-01, LDS-02 ou LDS-03 ; vide = celui du robot', style='Subtitle.TLabel').grid(
            row=8, column=2, sticky='w', padx=10)
        buttons = ttk.Frame(tab)
        buttons.pack(fill='x', pady=10)
        self.robot_connect = ttk.Button(buttons, text='Connecter', style='Primary.TButton', command=self.robot_connect_clicked)
        self.robot_connect.pack(side='left', padx=(0, 8))
        self.robot_bringup = ttk.Button(buttons, text='Lancer le bringup', command=self.robot_bringup_clicked)
        self.robot_bringup.pack(side='left', padx=(0, 8))
        self.robot_stop = ttk.Button(buttons, text='Arrêter le bringup', command=self.robot_stop_clicked)
        self.robot_stop.pack(side='left', padx=(0, 8))
        self.robot_disconnect = ttk.Button(buttons, text='Déconnecter', command=self.robot_disconnect_clicked)
        self.robot_disconnect.pack(side='left', padx=(0, 18))
        self.robot_camera_start = ttk.Button(buttons, text='Lancer la caméra', command=self.robot_camera_start_clicked)
        self.robot_camera_start.pack(side='left', padx=(0, 8))
        self.robot_camera_stop = ttk.Button(buttons, text='Arrêter la caméra', command=self.robot_camera_stop_clicked)
        self.robot_camera_stop.pack(side='left', padx=(0, 8))
        self.robot_camera_diag = ttk.Button(buttons, text='Diagnostic caméra', command=lambda: self._robot_run(self.robot.diagnose_camera))
        self.robot_camera_diag.pack(side='left')
        self.robot_status = ttk.Label(tab, text=self.robot.message, font=('DejaVu Sans', 11, 'bold'), wraplength=800)
        self.robot_status.pack(anchor='w', pady=(0, 6))
        self.robot_log = tk.Text(tab, height=12, wrap='word', bg='#0f1b2a', fg='#d6e4f2', relief='flat', padx=10, pady=8)
        self.robot_log.pack(fill='both', expand=True)
        self.robot_log.configure(state='disabled')
        self._robot_thread = None
        self._robot_closed = False
        self._robot_ui_queue = queue.Queue()
        for key in ('host', 'user', 'port'):
            self.robot_vars[key].trace_add('write', lambda *_: self._robot_identity_changed())
        self._robot_refresh_password_note()
        self.root.after_idle(self._robot_fill_camera_address)
        self._robot_refresh()

    # -- actions ---------------------------------------------------------------------------
    def _robot_settings(self):
        values = {k: v.get() for k, v in self.robot_vars.items() if k != 'password'}
        for key in ('port', 'ros_domain_id'):
            try:
                values[key] = int(values[key])
            except ValueError:
                pass
        values.update(self._robot_extra)
        return values

    def _robot_pick_key(self):
        chosen = filedialog.askopenfilename(title='Clé privée SSH', initialdir=str(ssh.Path.home()/'.ssh'))
        if chosen:
            self.robot_vars['key_file'].set(chosen)

    def _robot_ui(self, action):
        """Fait exécuter `action` par le fil de l'interface (appelable depuis un autre fil)."""
        self._robot_ui_queue.put(action)

    def _robot_run(self, work):
        if self._robot_thread is not None and self._robot_thread.is_alive():
            return
        self._robot_thread = threading.Thread(target=work, daemon=True)
        self._robot_thread.start()

    def _robot_ask_host_key(self, host, fingerprint):
        """Appelé depuis le thread de connexion : pose la question dans le thread de l'interface."""
        answer, done = [], threading.Event()
        def ask():
            answer.append(messagebox.askyesno(
                'Nouveau robot', f'Première connexion à {host}.\n\nEmpreinte : {fingerprint}\n\n'
                'Faire confiance à ce robot et l’enregistrer ?'))
            done.set()
        self._robot_ui(ask)
        done.wait(120)
        return bool(answer and answer[0])

    # -- mot de passe mémorisé (trousseau du système) --------------------------------------
    def _robot_refresh_password_note(self):
        """Interroge le trousseau hors du fil de l'interface (il peut demander un déverrouillage)."""
        available, reason = ssh.keyring_status()
        self.robot_remember_check.config(state='normal' if available else 'disabled')
        if not available:
            self.robot_password_note.config(text=reason)
            self.robot_forget.config(state='disabled')
            return
        settings = self._robot_settings()
        if not str(settings.get('host', '')).strip():
            self.robot_password_note.config(text='')
            self.robot_forget.config(state='disabled')
            return
        def work():
            status, value = ssh.recall_status(settings)
            self._robot_ui(lambda: self._robot_apply_saved(status, value))
        threading.Thread(target=work, daemon=True).start()

    def _robot_apply_saved(self, status, message):
        if self._robot_closed:
            return
        notes = {'ok': 'Un mot de passe est enregistré pour ce robot : laissez le champ vide.',
                 'absent': ('Aucun mot de passe enregistré pour ce robot : saisissez-le une fois, '
                            'avec la case cochée, puis Connecter.' if self.robot_vars['remember_password'].get() else ''),
                 'verrouille': message, 'erreur': message}
        self.robot_password_note.config(text=notes[status], foreground='#b52d3a' if status in ('verrouille', 'erreur') else '')
        if status == 'ok':
            self.robot_vars['remember_password'].set(True)     # un mot de passe est bien mémorisé
        self.robot_forget.config(state='normal' if status == 'ok' else 'disabled')

    def _robot_identity_changed(self):
        self._robot_fill_camera_address()
        if getattr(self, '_robot_note_timer', None) is not None:
            self.root.after_cancel(self._robot_note_timer)
        self._robot_note_timer = self.root.after(700, self._robot_refresh_password_note)

    def _robot_remember_toggled(self):
        if not self.robot_vars['remember_password'].get():
            self.robot_forget_clicked()
        ssh.save_settings(self._robot_settings())      # l'état de la case survit aux relances
        self._robot_refresh_password_note()

    def robot_forget_clicked(self):
        ssh.forget_password(self._robot_settings())
        self.robot_vars['remember_password'].set(False)
        self._robot_refresh_password_note()

    def _robot_fill_camera_address(self):
        """Reprend l'adresse du robot et le port caméra dans l'onglet « Caméra optionnelle »."""
        settings = self._robot_settings()
        if str(settings.get('host', '')).strip():
            self._robot_show_camera_url(ssh.camera_url(settings))

    def robot_connect_clicked(self):
        settings = self._robot_settings()
        typed = self.robot_vars['password'].get()
        remember = bool(self.robot_vars['remember_password'].get())
        def work():
            password = typed
            if not typed:
                status, value = ssh.recall_status(settings)
                if status in ('verrouille', 'erreur') and remember:
                    self.robot.output.put('[trousseau] '+value)
                    self.robot._set(ssh.ERROR, value)
                    return
                password = value if status == 'ok' else ''
            if self.robot.connect(settings, password, self._robot_ask_host_key):
                ssh.save_settings(settings)
                if remember and typed:
                    try:
                        ssh.remember_password(settings, typed)
                        self.robot.output.put('[trousseau] Mot de passe enregistré pour '+settings['user']+'@'+settings['host']+'.')
                    except Exception as exc:
                        self.robot.output.put('[trousseau] Mot de passe NON enregistré : '+str(exc))
                self._robot_ui(self._robot_refresh_password_note)
        self._robot_run(work)
        self.robot_vars['password'].set('')   # le mot de passe ne reste pas affiché

    def robot_bringup_clicked(self):
        settings = self._robot_settings()
        self._robot_run(lambda: self.robot.start_bringup(settings))

    def robot_stop_clicked(self):
        self._robot_run(self.robot.stop_bringup)

    def robot_camera_start_clicked(self):
        settings = self._robot_settings()
        def work():
            if self.robot.start_camera(settings):
                url = ssh.camera_url(settings)
                self._robot_ui(lambda: self._robot_show_camera_url(url))
        self._robot_run(work)

    def _robot_show_camera_url(self, url):
        """Pré-remplit l'onglet Caméra avec l'adresse du serveur qui vient d'être lancé."""
        entry = getattr(self, 'rtsp_entry', None)
        if entry is not None:
            entry.delete(0, 'end')
            entry.insert(0, url)

    def robot_camera_stop_clicked(self):
        self._robot_run(self.robot.stop_camera)

    def robot_disconnect_clicked(self):
        self._robot_run(self.robot.disconnect)

    def robot_shutdown(self):
        """À l'arrêt de l'atelier : coupe proprement le bringup lancé depuis ici."""
        self._robot_closed = True
        link = getattr(self, 'robot', None)
        if link is not None and link.state in (ssh.CONNECTED, ssh.BRINGUP):
            link.disconnect(quiet=True)

    # -- affichage -------------------------------------------------------------------------
    def _robot_refresh(self):
        if self._robot_closed:
            return
        while not self._robot_ui_queue.empty():
            try:
                self._robot_ui_queue.get_nowait()()
            except Exception:
                pass
        link = self.robot
        self.robot_status.config(text=link.message+('   ·   '+link.camera_message if link.camera_message else ''),
                                 foreground=STATE_COLORS.get(link.state, '#5a6b7b'))
        busy = self._robot_thread is not None and self._robot_thread.is_alive()
        connected = link.state in (ssh.CONNECTED, ssh.BRINGUP)
        def enable(widget, on):
            widget.config(state='normal' if on and not busy else 'disabled')
        enable(self.robot_connect, link.state != ssh.BRINGUP)
        enable(self.robot_bringup, link.state == ssh.CONNECTED)
        enable(self.robot_stop, link.state == ssh.BRINGUP)
        enable(self.robot_disconnect, connected)
        enable(self.robot_camera_start, connected and not link.camera_running)
        enable(self.robot_camera_stop, link.camera_running)
        enable(self.robot_camera_diag, connected)
        lines = []
        while not link.output.empty() and len(lines) < 200:
            lines.append(link.output.get())
        if lines:
            self.robot_log.configure(state='normal')
            self.robot_log.insert('end', '\n'.join(lines)+'\n')
            self.robot_log.see('end')
            self.robot_log.configure(state='disabled')
        self.root.after(250, self._robot_refresh)
