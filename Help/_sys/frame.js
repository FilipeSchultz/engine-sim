// Lógica da moldura (index.html): árvore, pesquisa e barra de ferramentas.
// Não precisa editar. O conteúdo da árvore fica em toc.js.

(function () {
  'use strict';

  var PAGINA_INICIAL = 'Inicio.html';
  // aceita subpastas: 'ECU/Funcoes.html'
  var NOME_VALIDO = /^([A-Za-z0-9_\-]+\/)*[A-Za-z0-9_\-]+\.html?(#[A-Za-z0-9_\-]*)?$/;

  var frame, frameIndexador, noPorArquivo = {}, listaPaginas = [];
  var indice = {}, indexando = false, indexadas = 0;
  var destaquePendente = null, tituloApp = document.title;

  // ---------- utilidades ----------
  function $(id) { return document.getElementById(id); }

  function normChar(c) {
    var n = c;
    if (n.normalize) { n = n.normalize('NFD').replace(/[\u0300-\u036f]/g, ''); }
    if (n.length !== 1) { n = c; }
    return n.toLowerCase();
  }
  function norm(s) {
    var r = '';
    for (var i = 0; i < s.length; i++) { r += normChar(s.charAt(i)); }
    return r;
  }
  function esc(s) {
    return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }
  function semAncora(a) { return String(a || '').split('#')[0]; }
  function enviarParaPagina(msg) {
    msg.helpMsg = 1;
    try { frame.contentWindow.postMessage(JSON.stringify(msg), '*'); } catch (e) { }
  }

  // ---------- navegação ----------
  function abrir(arquivo) {
    if (!NOME_VALIDO.test(arquivo)) { arquivo = PAGINA_INICIAL; }
    frame.src = arquivo;
  }

  function paginaInicialDaUrl() {
    var m = /[?&]p=([^&]*)/.exec(location.search);
    if (m) {
      var p = decodeURIComponent(m[1].replace(/\+/g, ' '));
      if (NOME_VALIDO.test(p)) { return p; }
    }
    return PAGINA_INICIAL;
  }

  // ---------- árvore ----------
  var ICONE_PASTA = '<svg viewBox="0 0 16 16" width="16" height="16"><path d="M1 3.5h5l1.5 1.5H15v8.5H1z" fill="#e8b84a" stroke="#b8871f" stroke-width="1"/></svg>';
  var ICONE_PAGINA = '<svg viewBox="0 0 16 16" width="16" height="16"><path d="M3 1.5h7l3 3v10H3z" fill="#fff" stroke="#6b7a90" stroke-width="1"/><path d="M5 7h6M5 9.5h6M5 12h4" stroke="#9aa6b8" stroke-width="1"/></svg>';

  function montarArvore(itens, ul, caminho) {
    for (var i = 0; i < itens.length; i++) {
      var it = itens[i], li = document.createElement('li');
      var temFilhos = it.filhos && it.filhos.length;
      li.className = temFilhos ? 'pasta fechada' : 'folha';

      var linha = document.createElement('div');
      linha.className = 'no';
      linha.innerHTML =
        '<span class="seta"></span>' +
        '<span class="icone">' + (temFilhos ? ICONE_PASTA : ICONE_PAGINA) + '</span>' +
        '<span class="rotulo">' + esc(it.titulo) + '</span>';
      li.appendChild(linha);
      ligarClique(linha, li, it);

      var novoCaminho = caminho.concat([it.titulo]);
      if (it.pagina) {
        noPorArquivo[semAncora(it.pagina)] = li;
        listaPaginas.push({ arquivo: semAncora(it.pagina), titulo: it.titulo, caminho: caminho.join(' › ') });
      }
      if (temFilhos) {
        var sub = document.createElement('ul');
        li.appendChild(sub);
        montarArvore(it.filhos, sub, novoCaminho);
      }
      ul.appendChild(li);
    }
  }

  function ligarClique(linha, li, it) {
    linha.onclick = function (ev) {
      var naSeta = ev.target && ev.target.className === 'seta';
      if (li.className.indexOf('pasta') >= 0 && (naSeta || !it.pagina)) {
        alternar(li);
      } else if (it.pagina) {
        if (li.className.indexOf('fechada') >= 0) { alternar(li); }
        abrir(it.pagina);
      }
    };
  }

  function alternar(li, abrirForcado) {
    var fechada = li.className.indexOf('fechada') >= 0;
    if (abrirForcado === true && !fechada) { return; }
    li.className = fechada ? li.className.replace('fechada', 'aberta') : li.className.replace('aberta', 'fechada');
  }

  function todasPastas(abrirTudo) {
    var pastas = $('arvore').querySelectorAll('li.pasta');
    for (var i = 0; i < pastas.length; i++) {
      pastas[i].className = 'pasta ' + (abrirTudo ? 'aberta' : 'fechada');
    }
  }

  function selecionarNaArvore(arquivo) {
    var ativos = $('arvore').querySelectorAll('.ativo');
    for (var i = 0; i < ativos.length; i++) { ativos[i].className = ativos[i].className.replace(/\s*ativo/, ''); }
    var li = noPorArquivo[arquivo];
    if (!li) { return; }
    for (var p = li.parentNode; p && p.id !== 'arvore'; p = p.parentNode) {
      if (p.tagName === 'LI') { alternar(p, true); }
    }
    var linha = li.firstChild;
    linha.className += ' ativo';
    var painel = $('painelConteudo'), topo = linha.offsetTop;
    if (topo < painel.scrollTop || topo > painel.scrollTop + painel.clientHeight - 30) {
      painel.scrollTop = topo - 60;
    }
  }

  // ---------- abas ----------
  function mostrarAba(nome) {
    $('abaConteudo').className = 'aba' + (nome === 'conteudo' ? ' sel' : '');
    $('abaPesquisa').className = 'aba' + (nome === 'pesquisa' ? ' sel' : '');
    $('painelConteudo').style.display = nome === 'conteudo' ? 'block' : 'none';
    $('painelPesquisa').style.display = nome === 'pesquisa' ? 'flex' : 'none';
    if (nome === 'pesquisa') { $('campoBusca').focus(); iniciarIndexacao(); }
  }

  // ---------- indexação (carrega cada página num iframe oculto) ----------
  function iniciarIndexacao() {
    if (indexando || indexadas) { return; }
    indexando = true;
    frameIndexador = document.createElement('iframe');
    frameIndexador.style.display = 'none';
    document.body.appendChild(frameIndexador);
    indexarProxima(0);
  }

  var timerIndexacao = null, posIndexacao = 0;
  function indexarProxima(pos) {
    posIndexacao = pos;
    clearTimeout(timerIndexacao);
    if (pos >= listaPaginas.length) {
      indexando = false;
      indexadas = listaPaginas.length;
      document.body.removeChild(frameIndexador);
      frameIndexador = null;
      atualizarStatusIndice();
      pesquisar();
      return;
    }
    atualizarStatusIndice();
    frameIndexador.src = listaPaginas[pos].arquivo;
    // página inexistente ou sem pagina.js: segue adiante depois de 4 s
    timerIndexacao = setTimeout(function () { indexarProxima(pos + 1); }, 4000);
  }

  function atualizarStatusIndice() {
    var st = $('statusBusca');
    if (indexando) {
      st.innerHTML = 'Indexando páginas… ' + posIndexacao + ' de ' + listaPaginas.length;
    } else if (!$('campoBusca').value) {
      st.innerHTML = listaPaginas.length + ' páginas disponíveis para pesquisa.';
    }
  }

  function registrarPagina(d) {
    var reg = indice[d.arquivo] || {};
    reg.arquivo = d.arquivo;
    reg.tituloPagina = d.titulo;
    reg.texto = d.texto || '';
    reg.textoNorm = norm(reg.texto);
    indice[d.arquivo] = reg;
  }

  // ---------- pesquisa ----------
  function termosDe(consulta) {
    var partes = norm(consulta).split(/\s+/), r = [];
    for (var i = 0; i < partes.length; i++) { if (partes[i].length >= 2) { r.push(partes[i]); } }
    return r;
  }

  function contar(txt, termo) {
    var n = 0, p = txt.indexOf(termo);
    while (p >= 0 && n < 50) { n++; p = txt.indexOf(termo, p + termo.length); }
    return n;
  }

  function trecho(texto, textoNorm, termos) {
    var p = -1;
    for (var i = 0; i < termos.length; i++) {
      var q = textoNorm.indexOf(termos[i]);
      if (q >= 0 && (p < 0 || q < p)) { p = q; }
    }
    if (p < 0) { return esc(texto.substring(0, 140)) + (texto.length > 140 ? '…' : ''); }
    var ini = Math.max(0, p - 60), fim = Math.min(texto.length, p + 110);
    var t = texto.substring(ini, fim), tn = textoNorm.substring(ini, fim), marcas = [];
    for (var j = 0; j < termos.length; j++) {
      var k = tn.indexOf(termos[j]);
      while (k >= 0) { marcas.push([k, k + termos[j].length]); k = tn.indexOf(termos[j], k + 1); }
    }
    marcas.sort(function (a, b) { return a[0] - b[0]; });
    var html = '', pos = 0;
    for (var m = 0; m < marcas.length; m++) {
      if (marcas[m][0] < pos) { continue; }
      html += esc(t.substring(pos, marcas[m][0])) + '<b>' + esc(t.substring(marcas[m][0], marcas[m][1])) + '</b>';
      pos = marcas[m][1];
    }
    html += esc(t.substring(pos));
    return (ini > 0 ? '…' : '') + html + (fim < texto.length ? '…' : '');
  }

  function pesquisar() {
    var consulta = $('campoBusca').value, termos = termosDe(consulta), res = [];
    var caixa = $('resultados');
    caixa.innerHTML = '';
    if (!termos.length) { atualizarStatusIndice(); return; }

    for (var i = 0; i < listaPaginas.length; i++) {
      var pg = listaPaginas[i], reg = indice[pg.arquivo];
      var textoTitulo = norm(pg.titulo + ' ' + (reg ? reg.tituloPagina : ''));
      var corpo = reg ? reg.textoNorm : '', pontos = 0, todos = true;
      for (var t = 0; t < termos.length; t++) {
        var noTitulo = textoTitulo.indexOf(termos[t]) >= 0, noCorpo = contar(corpo, termos[t]);
        if (!noTitulo && !noCorpo) { todos = false; break; }
        pontos += (noTitulo ? 20 : 0) + noCorpo;
      }
      if (todos) { res.push({ pg: pg, reg: reg, pontos: pontos }); }
    }
    res.sort(function (a, b) { return b.pontos - a.pontos; });

    $('statusBusca').innerHTML = (indexando ? 'Indexando… resultados parciais. ' : '') +
      res.length + (res.length === 1 ? ' resultado' : ' resultados');

    for (var r = 0; r < res.length; r++) {
      var div = document.createElement('div');
      div.className = 'resultado';
      div.innerHTML =
        '<div class="res-titulo">' + esc(res[r].pg.titulo) + '</div>' +
        (res[r].pg.caminho ? '<div class="res-caminho">' + esc(res[r].pg.caminho) + '</div>' : '') +
        (res[r].reg ? '<div class="res-trecho">' + trecho(res[r].reg.texto, res[r].reg.textoNorm, termos) + '</div>' : '');
      ligarResultado(div, res[r].pg.arquivo, termos);
      caixa.appendChild(div);
    }
  }

  function ligarResultado(div, arquivo, termos) {
    div.onclick = function () {
      var sel = $('resultados').querySelectorAll('.resultado.sel');
      for (var i = 0; i < sel.length; i++) { sel[i].className = 'resultado'; }
      div.className = 'resultado sel';
      destaquePendente = { arquivo: arquivo, termos: termos };
      abrir(arquivo);
    };
  }

  // ---------- mensagens das páginas ----------
  function aoReceberMensagem(ev) {
    var d;
    try { d = JSON.parse(ev.data); } catch (e) { return; }
    if (!d || !d.helpMsg || d.tipo !== 'pagina') { return; }

    if (frameIndexador && ev.source === frameIndexador.contentWindow) {
      registrarPagina(d);
      indexarProxima(posIndexacao + 1);
      return;
    }
    if (ev.source !== frame.contentWindow) { return; }

    registrarPagina(d);
    selecionarNaArvore(d.arquivo);
    document.title = d.titulo === tituloApp ? tituloApp : d.titulo + ' - ' + tituloApp;
    $('tituloPagina').innerHTML = esc(d.titulo === tituloApp ? 'Início' : d.titulo);
    try { history.replaceState(null, '', 'index.html?p=' + encodeURIComponent(d.arquivo)); } catch (e) { }

    if (destaquePendente && destaquePendente.arquivo === d.arquivo) {
      enviarParaPagina({ tipo: 'destacar', termos: destaquePendente.termos });
    }
    destaquePendente = null;
  }

  // ---------- divisor arrastável ----------
  function ligarDivisor() {
    var div = $('divisor'), lateral = $('lateral'), capa = $('capaArraste'), arrastando = false;
    div.onmousedown = function (e) { arrastando = true; capa.style.display = 'block'; e.preventDefault(); };
    document.addEventListener('mousemove', function (e) {
      if (!arrastando) { return; }
      lateral.style.width = Math.max(180, Math.min(600, e.clientX)) + 'px';
    });
    document.addEventListener('mouseup', function () { arrastando = false; capa.style.display = 'none'; });
  }

  // ---------- inicialização ----------
  function iniciar() {
    frame = $('frameConteudo');
    montarArvore(window.HELP_TOC || [], $('arvore'), []);
    window.addEventListener('message', aoReceberMensagem);

    $('abaConteudo').onclick = function () { mostrarAba('conteudo'); };
    $('abaPesquisa').onclick = function () { mostrarAba('pesquisa'); };
    $('btnVoltar').onclick = function () { history.back(); };
    $('btnAvancar').onclick = function () { history.forward(); };
    $('btnInicio').onclick = function () { abrir(PAGINA_INICIAL); };
    $('marca').onclick = function () { abrir(PAGINA_INICIAL); };
    $('btnImprimir').onclick = function () { enviarParaPagina({ tipo: 'imprimir' }); };
    $('btnExpandir').onclick = function () { todasPastas(true); };
    $('btnRecolher').onclick = function () { todasPastas(false); };

    var espera = null;
    $('campoBusca').oninput = function () { clearTimeout(espera); espera = setTimeout(pesquisar, 200); };
    $('campoBusca').onkeydown = function (e) {
      if (e.keyCode === 13) { clearTimeout(espera); pesquisar(); }
      if (e.keyCode === 27) { this.value = ''; pesquisar(); }
    };

    ligarDivisor();
    abrir(paginaInicialDaUrl());
    // indexa em segundo plano logo após abrir, para a pesquisa ficar pronta
    setTimeout(iniciarIndexacao, 800);
  }

  document.addEventListener('DOMContentLoaded', iniciar);
})();
